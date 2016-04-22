import MultiNEAT as NEAT
from robot_controller2 import RobotController
params = NEAT.Parameters()
params.PopulationSize = 100

genome = NEAT.Genome(0, 643, 0, 2, False, NEAT.ActivationFunction.UNSIGNED_SIGMOID, NEAT.ActivationFunction.UNSIGNED_SIGMOID, 0, params)     
pop = NEAT.Population(genome, params, True, 1.0, 0)
pop.RNG.Seed(0)


def evaluate(genome):

    # this creates a neural network (phenotype) from the genome

    net = NEAT.NeuralNetwork()
    genome.BuildPhenotype(net)

    # let's input just one pattern to the net, activate it once and get the output

    def driver(sensor_input):
        print sensor_input[1]
        sensor_input.append(1)
        net.Input = sensor_input
        net.Activate()
        (left, right) = net.Output()
        return (left, right)
    
    r = RobotController(True)
    (fitness, _, _) = r.run(driver, 20)
    return fitness
    
genome_list = NEAT.GetGenomeList(pop)
fitness = evaluate(genome_list[33])
print fitness
