#!/usr/bin/python
import argparse
from visualization_utils import *

if __name__ == '__main__':
    print '*------ STARTING VISUALIZE INSTANCE ENTROPY ------*'
    
    # Parse the input
    parser = argparse.ArgumentParser(description='Visualize the entropy values in an instance directory.')
    parser.add_argument('directory', type=str,
                       help='the directory containing the files with the data to process and visualize')
    args = parser.parse_args()
    print 'Input directory: ' + args.directory
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    visualize_instance_entropy(args.directory)
    
    plt.show()
    
    print '*--------------- FINISHED ---------------*'