#!/usr/bin/python
import argparse
from visualization_utils import *
from matplotlib import rc
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    print '*------ STARTING SEARCH INVALID OBJECTS ------*'
    
    # Parse the input
    parser = argparse.ArgumentParser(description='Search through results directories and determine which objects ar invalid. Save the results to file.')
    parser.add_argument('directory', type=str,
                       help='the directory containing the sub directories of the actual files to search')
    parser.add_argument('number_classes', type=int,
                       help='the number of classes used in the training set')
    parser.add_argument('number_objects', type=int,
                       help='the number of objects in this dataset')
    args = parser.parse_args()
    print 'Input directory: ' + args.directory
    print 'Number of classes: ' + str(args.number_classes)
    print 'Number of objects: ' + str(args.number_objects)
    
    save_ignore_objects_file(args.directory, args.number_classes, args.number_objects)
    
    print '*--------------- FINISHED ---------------*'