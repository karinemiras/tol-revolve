# Offline evolution scheme
# - We use a population of constant size 10
# - Each robot is evaluated for 20 seconds, though we may vary this number
# - The average speed during this evaluation is the fitness
# - We do parent selection using a binary tournament: select two parents at
#   random, the one with the best fitness is parent 1, repeat for parent 2.
# - Using this mechanism, we generate 10 children
# - After evaluation of the children, we either do:
# -- Plus scheme, sort *all* robots by fitness
# -- Comma scheme, get rid of the parents and continue with children only
from __future__ import absolute_import
import sys
import time
import os
import shutil
import random
import csv
import itertools
import logging
import trollius
from trollius import From, Return

from sdfbuilder import Pose
from sdfbuilder.math import Vector3

from revolve.util import wait_for

here = os.path.dirname(os.path.abspath(__file__))
tol_path = os.path.abspath(os.path.join(here, '..', '..'))
sys.path.append(tol_path)

from tol.manage.robot import Robot
from tol.config import parser
from tol.manage import World
from tol.logging import logger, output_console
from tol.util.analyze import list_extremities, count_joints, count_motors, count_extremities, count_connections

path_to_environ_indirect='/Users/karinemiras/projects/coevolution-revolve/l' \
                         '-system/build/lib'
sys.path.append(path_to_environ_indirect)
import lsystem_python
import math
import numpy as np




# Log output to console
output_console()
logger.setLevel(logging.DEBUG)

# Add offline evolve arguments
parser.add_argument(
    '--population-size',
    default=10, type=int,
    help="Population size in each generation."
)

parser.add_argument(
    '--num-children',
    default=10, type=int,
    help="The number of children produced in each generation."
)


def str2bool(v):
    return v.lower() == "true" or v == "1"

parser.add_argument(
    '--keep-parents',
    default=True, type=str2bool,
    help="Whether or not to discard the parents after each generation. This determines the strategy, + or ,."
)

parser.add_argument(
    '--num-generations',
    default=200, type=int,
    help="The number of generations to simulate."
)

parser.add_argument(
    '--disable-evolution',
    default=False, type=str2bool,
    help="Useful as a baseline test - if set to true, new robots are generated"
         " every time rather than evolving them."
)

parser.add_argument(
    '--disable-selection',
    default=False, type=str2bool,
    help="Useful as a different baseline test - if set to true, robots reproduce,"
         " but parents are selected completely random."
)

parser.add_argument(
    '--disable-fitness',
    default=False, type=str2bool,
    help="Another baseline testing option, sorts robots randomly rather "
         "than selecting the top pairs. This only matters if parents are kept."
)

parser.add_argument(
    '--num-evolutions',
    default=30, type=int,
    help="The number of times to repeat the experiment."
)

parser.add_argument(
    '--evaluation-threshold',
    default=30.0, type=float,
    help="Maximum number of seconds one evaluation can take before the "
         "decision is made to restart from snapshot. The assumption is "
         "that the world may have become slow and restarting will help."
)


class OfflineEvoManager(World):
    """
    Extended world manager for the offline evolution script
    """

    def __init__(self, conf, _private):
        """

        :param conf:
        :param _private:
        :return:
        """
        super(OfflineEvoManager, self).__init__(conf, _private)

        self._snapshot_data = {}

        # Output files
        csvs = {
            'generations': ['run', 'gen', 'robot_id', 'vel', 'dvel', 'fitness', 't_eval'],
            'robot_details': ['robot_id', 'extremity_id', 'extremity_size',
                              'joint_count', 'motor_count']
        }
        self.csv_files = {k: {'filename': None, 'file': None, 'csv': None,
                              'header': csvs[k]}
                          for k in csvs}

        self.current_run = 0

        if self.output_directory:
            for k in self.csv_files:
                fname = os.path.join(self.output_directory, k + '.csv')
                self.csv_files[k]['filename'] = fname
                if self.do_restore:
                    shutil.copy(fname + '.snapshot', fname)
                    f = open(fname, 'ab', buffering=1)
                else:
                    f = open(fname, 'wb', buffering=1)

                self.csv_files[k]['file'] = f
                self.csv_files[k]['csv'] = csv.writer(f, delimiter=',')

                if not self.do_restore:
                    self.csv_files[k]['csv'].writerow(self.csv_files[k]['header'])

    def robots_header(self):
        return Robot.header()

    @classmethod
    @trollius.coroutine
    def create(cls, conf):
        """
        Coroutine to instantiate a Revolve.Angle WorldManager
        :param conf:
        :return:
        """
        self = cls(_private=cls._PRIVATE, conf=conf)
        yield From(self._init())
        raise Return(self)

    @trollius.coroutine
    def create_snapshot(self):
        """
        Copy the generations file in the snapshot
        :return:
        """
        ret = yield From(super(OfflineEvoManager, self).create_snapshot())
        if not ret:
            raise Return(ret)

        for k in self.csv_files:
            entry = self.csv_files[k]
            if entry['file']:
                entry['file'].flush()
                shutil.copy(entry['filename'], entry['filename'] + '.snapshot')

    @trollius.coroutine
    def get_snapshot_data(self):
        """
        :return:
        """
        data = yield From(super(OfflineEvoManager, self).get_snapshot_data())
        data.update(self._snapshot_data)
        raise Return(data)

    @trollius.coroutine
    def evaluate_pair(self, tree, bbox, parents=None):
        """
        Evaluates a single robot tree.
        :param tree:
        :param bbox:
        :param parents:
        :return: Evaluated Robot object
        """
        # Pause the world just in case it wasn't already
        yield From(wait_for(self.pause(True)))

        args = parser.parse_args()
        pose = Pose(position=Vector3(0, 0, args.init_z))

        fut = yield From(self.insert_robot(tree, pose, parents=parents))
        robot = yield From(fut)

        max_age = self.conf.evaluation_time + self.conf.warmup_time

        # Unpause the world to start evaluation
        yield From(wait_for(self.pause(False)))

        before = time.time()

        while True:
            if robot.age() >= max_age:
                break

            # Sleep for the pose update frequency, which is about when
            # we expect a new age update.
            yield From(trollius.sleep(1.0 / self.state_update_frequency))

        yield From(wait_for(self.delete_robot(robot)))
        yield From(wait_for(self.pause(True)))

        diff = time.time() - before
        if diff > self.conf.evaluation_threshold:
            sys.stderr.write("Evaluation threshold exceeded, shutting down with nonzero status code.\n")
            sys.stderr.flush()
            sys.exit(15)

        raise Return(robot)

    @trollius.coroutine
    def evaluate_population(self, trees, bboxes, validity_list):
        """
        :param trees:
        :param bboxes:
        :return:
        """
        parents = [None for _ in trees]

        pairs = []
        print("Evaluating population...")
        ids = 0
        for tree, bbox, par in itertools.izip(trees, bboxes, parents):

            print("Evaluating individual...")
            before = time.time()
            robot = yield From(self.evaluate_pair(tree, bbox, par))

            #does not read ids of invalids
            while validity_list[ids][1] == '0':
                ids +=1
            robot.robot.id = int(validity_list[ids][0])
            ids +=1

            pairs.append((robot, time.time() - before))
            print("Done.")

        print("Done evaluating population.")
        raise Return(pairs)

    @trollius.coroutine
    def produce_generation(self, parents):
        """
        Produce the next generation of robots from
        the current.
        :param parents:
        :return:
        """
        print("Producing generation...")
        trees = []
        bboxes = []
        parent_pairs = []

        while len(trees) < self.conf.num_children:
            print("Producing individual...")
            if self.conf.disable_selection:
                p1, p2 = random.sample(parents, 2)
            else:
                p1, p2 = select_parents(parents, self.conf)

            for j in xrange(self.conf.max_mating_attempts):
                pair = yield From(self.attempt_mate(p1, p2))
                if pair:
                    trees.append(pair[0])
                    bboxes.append(pair[1])
                    parent_pairs.append((p1, p2))
                    break

            print("Done.")

        print("Done producing generation.")
        raise Return(trees, bboxes, parent_pairs)


    @trollius.coroutine
    def run(self):

        """
        :return:
        #update:karinemiras
        """
        args = parser.parse_args()

        # run an experiment
        if args.exp_test == "e" :

            # checks if there any experiment to recover
            recovery_path = '../../../l-system/experiments/'+args.experiment_name+'/evolutionstate.txt'
            if os.path.exists(recovery_path):
                with open(recovery_path) as f:
                    file = f.read().split(" ")

                load_experiment = 1
                init_generation = int(file[0])+1
            else:
                load_experiment = 0
                init_generation = 1

            # instantiates evolution C++ class
            evolve_generation = lsystem_python.EvolutionIndirect(args.experiment_name,
                                                                 "../../../l-system/")

            # setup up new evolution if it is not a recovery step
            if load_experiment == 0:
                evolve_generation.setupEvolution()

            for generation in range(init_generation, args.generations+1):

                validity_list = []

                # prepares genotypes
                evolve_generation.runExperiment_part1(generation,
                                                      load_experiment)

                # load experiment applies only for the initial generation
                load_experiment = 0

                with open('../../../l-system/experiments/'
                                  +args.experiment_name
                                  +'/offspringpop'+str(generation)
                                  +'/validity_list.txt') as f:
                    file = f.read().splitlines()
                for line in range(0,len(file)):
                    l = file[line].split(" ")
                    validity_list.append([l[0],l[1]])

                # simulation
                trees, bboxes = yield From(self.generate_population(
                    args.experiment_name, generation, validity_list))
                pairs = yield From(self.evaluate_population(trees, bboxes,
                                                            validity_list))

                # updates fitnesses
                for robot, t_eval in pairs:
                    evolve_generation.saveLocomotionFitness(str(robot.robot.id),
                                                  robot.displacement_velocity())


                # post processing of results
                evolve_generation.runExperiment_part2(generation)


        # testing a robot
        else:

            validity_list = []

            # best of last 10 generations
            if args.exp_test == "t1" :

                with open('../../../l-system/experiments/'
                                  +args.experiment_name
                                  +'/evolution.txt') as f:
                    file_genomes = f.read().splitlines()
                for line in range(1,len(file_genomes)-1):
                    l1 = file_genomes[line].split(" ")
                    l2 = file_genomes[line+1].split(" ")

                    if(l1[7] != l2[7] or line == len(file_genomes)-2):
                        validity_list.append([l1[7],'1'])

            # 10 best of the last generation
            if args.exp_test == "t2" :

                genomes_list = []
                fitness_list = {}

                with open('../../../l-system/experiments/'+args.experiment_name +'/history.txt') as f:
                    file_genomes = f.read().splitlines()

                for line in range(1,len(file_genomes)):

                    array_line = file_genomes[line].split(" ")
                    fitness_list[array_line[1]] = array_line[4]

                for f in os.listdir('../../../l-system/experiments/'
                                            +args.experiment_name
                                    +'/selectedpop'+str(args.generations)):

                    filename = f.split("_")
                    idgenome = filename[1]

                    genomes_list.append([idgenome, float(fitness_list[
                        idgenome])])

                genomes_list = sorted(genomes_list,key=lambda x: x[1])
         
                for i in range(0,len(genomes_list)):
                     validity_list.append([genomes_list[i][0],'1'])

            validity_list = validity_list[len(validity_list)-8:
                                          len(validity_list)]

            # 10 best of the last generation
            if args.exp_test == "t3" :
                validity_list.append(['4404','1'])


            for i in range(0,len(validity_list)):

                genome = []
                genome.append(validity_list[i])

                trees, bboxes = yield From(self.generate_population(
                    args.experiment_name, self.getGeneration_genome(
                        validity_list[i][0]), genome))

                pairs = yield From(self.evaluate_population(trees, bboxes,
                                                            genome))
                for robot, t_eval in pairs:
                    print("id: "+str(robot.robot.id))
                    print("fitness: "+str(robot.displacement_velocity()))

        yield From(self.teardown())


    def getGeneration_genome(self, idgenome):

        args = parser.parse_args()

        generation_genome = 0
        offspring_size = args.pop_size * args.offspring_prop

        # generation of the genome can be found by its id, considering the size of the population and the offspring
        if (args.offspring_prop == 1):
            generation_genome =  math.trunc( int(idgenome) / args.pop_size) + 1

        else:

            generation_genome =   math.trunc((int(idgenome) - offspring_size)/ offspring_size) + 1


        if (generation_genome == 0):
            generation_genome = 1

        return generation_genome


    @trollius.coroutine
    def teardown(self):
        """
        :return:
        """
        yield From(super(OfflineEvoManager, self).teardown())
        for k in self.csv_files:
            if self.csv_files[k]['file']:
                self.csv_files[k]['file'].close()


def select_parent(parents, conf):
    """
    Select a parent using a binary tournament.
    :param parents:
    :param conf: Configuration object
    :return:
    """
    return sorted(random.sample(parents, conf.tournament_size), key=lambda r: r.fitness())[-1]


def select_parents(parents, conf):
    """
    :param parents:
    :param conf: Configuration object
    :return:
    """
    p1 = select_parent(parents, conf)
    p2 = select_parent(list(parent for parent in parents if parent != p1), conf)
    return p1, p2


@trollius.coroutine
def run():
    """
    :return:
    """
    conf = parser.parse_args()
    world = yield From(OfflineEvoManager.create(conf))
    yield From(world.run())


def main():
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got Ctrl+C, shutting down.")


if __name__ == '__main__':
    main()
