#!/usr/bin/python
import argparse
from visualization_utils import *

if __name__ == '__main__':
    print '*------ STARTING VISUALIZE PLANNERS ------*'
    
    # Parse the input
    parser = argparse.ArgumentParser(description='Process and visualize the results files for all planners.')
    parser.add_argument('directory', type=str,
                       help='the directory containing the directories of results for different planners')
    parser.add_argument('--ignore_plans', type=str, nargs='+', default=[''],
                       help='list of plans to ignore for visualizing the results')
    parser.add_argument('--remove_invalid_objects', '-r', action='store_true',
                       help='flag to ignore invalid objects')
    args = parser.parse_args()
    print 'Input directory: ' + args.directory
    print 'Ignore plans: ' + ' '.join(args.ignore_plans)
    if args.remove_invalid_objects == True:
        print 'Ignoring invalid objects: TRUE'
    else:
        print 'Ignoring invalid objects: FALSE'
    
    res = read_plan_set_directory(args.directory, True, args.remove_invalid_objects)
    print 'Number of planners: ' + str(len(res))
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    visualize_planners(res, args.ignore_plans)
    plt.show()
    
    print '*--------------- FINISHED ----------------*'