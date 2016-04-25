import MultiNEAT as NEAT
from robot_controller2 import RobotController
import numpy as np


params = NEAT.Parameters()
params.PopulationSize = 10
params.MutateNeuronActivationTypeProb  = 0.1
params.ActivationFunction_SignedSigmoid_Prob = 0.07
params.ActivationFunction_UnsignedSigmoid_Prob = 0.07
params.ActivationFunction_Tanh_Prob = 0.7
params.ActivationFunction_TanhCubic_Prob = 0.07
params.ActivationFunction_SignedStep_Prob = 0.07
params.ActivationFunction_UnsignedStep_Prob = 0.07
params.ActivationFunction_SignedGauss_Prob = 0.07
params.ActivationFunction_UnsignedGauss_Prob = 0.07
params.ActivationFunction_Abs_Prob = 0.07
params.ActivationFunction_SignedSine_Prob = 0.07
params.ActivationFunction_UnsignedSine_Prob = 0.07
params.ActivationFunction_SignedSquare_Prob = 0.07
params.ActivationFunction_UnsignedSquare_Prob = 0.07
params.ActivationFunction_Linear_Prob = 0.07

genome = NEAT.Genome(0, 643, 2, 2, True, NEAT.ActivationFunction.RELU, NEAT.ActivationFunction.LINEAR, 1, params)     
pop = NEAT.Population(genome, params, True, 1.0, 1)
#pop.RNG.Seed(1)


def evaluate(genome):

    # this creates a neural network (phenotype) from the genome

    net = NEAT.NeuralNetwork()
    genome.BuildPhenotype(net)

    # let's input just one pattern to the net, activate it once and get the output

    def driver(sensor_input):
        sensor_input.append(1)
        #a = np.array(sensor_input)
        net.Input(np.array(sensor_input))
        net.Activate()
        (left, right) = net.Output()
        #print (left, right)
        return (left, right)
    
    r = RobotController()
    (fitness, _, _) = r.run(driver, 10)
    return (40 - fitness)
    
for generation in range(5):
    genome_list = NEAT.GetGenomeList(pop)
    for genome in genome_list:
        fitness = evaluate(genome)
        print fitness
        genome.SetFitness(fitness)
    pop.Epoch()

