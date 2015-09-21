#!/usr/bin/python
import argparse
from visualization_utils import *

if __name__ == '__main__':
    print '*------ STARTING VISUALIZE MINIMUM VIEWS ------*'
    
    # Parse the input
    parser = argparse.ArgumentParser(description='Process and visualize the results files for all planners.') 
    parser.add_argument('directory', type=str,
                       help='the directory containing the directories of different simulation runs, \
                        each sub directory contains directories of results')
    parser.add_argument('--confidence', type=float, default=0.0,
                        help='the confidence level to for how many views are needed to achieve')
    parser.add_argument('--class_type', type=str, default="",
                        help='the true class of the object (only applies if one object visible in scene)')
    parser.add_argument('--ignore_plans', type=str, nargs='+', default=[],
                       help='list of plans to ignore for visualizing the results')
    parser.add_argument('--ignore_sets', type=int, nargs='+', default=[],
                       help='list of sets to ignore for visualizing the results, -1 is for the index_min_entropy set')
    parser.add_argument('--single_object', '-s', action='store_true',
                       help='if only one object in the set')
    parser.add_argument('--remove_invalid_objects', '-r', action='store_true',
                       help='flag to ignore invalid objects')
    args = parser.parse_args()
    print 'Input directory: ' + args.directory
    print 'Confidence: ' + str(args.confidence)
    print 'Class: ' + str(args.class_type)
    print 'Ignore plans: ' + ' '.join(args.ignore_plans)
    print 'Ignore sets: ' + str(args.ignore_sets)
    if args.remove_invalid_objects == True:
        print 'Ignoring invalid objects: TRUE'
    else:
        print 'Ignoring invalid objects: FALSE'
    
    res = read_simulation_set_directory(args.directory, args.remove_invalid_objects)
    print 'Number of simulations: ' + str(len(res))
    
    # Convert ignore sets to strings
    ignore_sets_strs = []
    for i in args.ignore_sets:
        ignore_sets_strs.append(str(i))
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    if args.single_object == True:
        visualize_minimum_views_one_object(res, args.confidence, args.class_type, args.ignore_plans, ignore_sets_strs)
    else:
        visualize_minimum_views(res, args.confidence, args.class_type, args.ignore_plans, ignore_sets_strs)
    plt.show()
    
    print '*----------------- FINISHED ------------------*'