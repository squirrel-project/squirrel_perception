#!/usr/bin/python
import argparse
from visualization_utils import *

if __name__ == '__main__':
    print '*------ STARTING VISUALIZE RESULTS ------*'
    
    # Parse the input
    parser = argparse.ArgumentParser(description='Process and visualize the single set of results in a specified directory.')
    parser.add_argument('training_directory', type=str,
                       help='the training directory containing the classes')
    parser.add_argument('directory', type=str,
                       help='the directory containing the files with the data to process and visualize')
    parser.add_argument('--class_type', type=str, default="",
                        help='the true class of the object (only applies if one object visible in scene)')
    parser.add_argument('--ignore_objects', type=int, nargs='+', default=[],
                       help='list of object indices to ignore for visualizing the results')
    parser.add_argument('--remove_invalid_objects', '-r', action='store_true',
                       help='flag to ignore invalid objects')
    args = parser.parse_args()
    print 'Training directory: ' + args.training_directory
    print 'Input directory: ' + args.directory
    print 'Class: ' + str(args.class_type)
    ignore_objects = []
    number_classes = -1
    number_objects = -1
    if len(args.ignore_objects) > 0:
        if len(args.ignore_objects) < 3:
            print "If ignoring objects then you must enter the number of classes ", \
                  "and number of objects at the end of the list"
            print 'e.g. --ignore_objects x y z number_classes number_objects'
            print 'Need at least 3 numbers!'
            sys.exit()
        else:
            ignore_objects = args.ignore_objects[0:-3]
            number_classes = int(args.ignore_objects[-2])
            number_objects = int(args.ignore_objects[-1])
    print 'Ignore objects: ' + ' '.join(map(str, ignore_objects))
    if len(ignore_objects) > 0:
        print 'Number classes: ' + str(number_classes)
        print 'Number of objects: ' + str(number_objects)
    
    # Find an ignore objects file in the directory
    if len(ignore_objects) == 0:
        if args.remove_invalid_objects == True:
            print 'Ignoring invalid objects: TRUE'
        else:
            print 'Ignoring invalid objects: FALSE'
        
        if args.remove_invalid_objects == True:
            ignore_objects, number_classes, number_objects = find_ignore_objects_file(args.directory)
            if len(ignore_objects) > 0:
                print 'Found ignore objects file!'
                print 'Ignore objects: ' + ' '.join(map(str, ignore_objects))
                print 'Number classes: ' + str(number_classes)
                print 'Number of objects: ' + str(number_objects)
    
    number_classes = int(number_classes)
    number_objects = int(number_objects)
    res, profiles = read_directory(args.directory, True, ignore_objects, number_classes, number_objects)
    print 'Number of observations: ' + str(len(res))
    
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    
    visualize_entropy_results(res, profiles, args.ignore_objects)
    visualize_object_profiles(profiles, args.training_directory, args.class_type)
    plt.show()
    
    print '*--------------- FINISHED ---------------*'