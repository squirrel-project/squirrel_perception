#!/usr/bin/python
from __future__ import division
import numpy as np
import sys
import os.path
from math import log
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy import interpolate
import colorsys

_SCALE_MAX_CONFIDENCE = 0.95
_RESULTS_FILE = 'results.txt'
_OBJECT_FILE_PREFIX = 'probabilities_'
_PLAN_ORDER = ['worst_to_best_entropy', 'worst_to_best_prob', \
               'random', 'nearest_area','nearest_max_prob',  'nearest_min_entropy', \
               'max_area','max_class_prob',  'min_class_entropy', \
               'max_area_unoccluded', 'max_class_prob_unoccluded',  'min_class_entropy_unoccluded',\
               'best_to_worst_prob', 'best_to_worst_entropy']
_INDEX_MIN_ENT = 'index_min_ent'
_IGNORE_OBJECTS_FILENAME = 'ignore_objects.txt'
_OBJECT_PROBABILITIES_FILENAME = 'probabilities_0.txt'
_LATEX_TABLE_FILENAME = '/home/tpat8946/table_column.txt'
_MIN_VIEWS_FOR_VALID_OBJECT = 2


class results_profile:
    def __init__(self):
        # Initialize to empty
        self._location = [0,0,0]  # robot location
        self._number_objects = -1  # number of objects
        self._entropy = -1  # total entropy
        self._average_entropy = -1  # average entropy
        self._entropy_max = -1  # entropy plus the entropy of unseen objects
        self._average_entropy_max = -1  # average entropy plus the entropy of unseen objects
        
    def set(self, data):
        # Set the values from the data
        # 0 = x position
        # 1 = y position
        # 2 = z position
        self._location = [float(data[0]), float(data[1]), float(data[2])]
        # 3 = number of objects
        self._number_objects = int(data[3])
        # 4 = entropy
        self._entropy = float(data[4])
        # 5 = entropy max
        self._entropy_max = float(data[5])
        # if the number of objects is larger than 0 then can compute the averages
        if self._number_objects > 0:
            self._average_entropy = self._entropy / self._number_objects
            self._average_entropy_max = self._entropy_max / self._number_objects
    
    def __str__(self):
        return 'Location: ' + str(self._location[0]) + ' ' + \
                str(self._location[1]) + ' ' + str(self._location[2]) + '\n' + \
                'Number of objects: ' + str(self._number_objects) + '\n' + \
                'Entropy: ' + str(self._entropy) + '\n' + \
                'Average Entropy: ' + str(self._average_entropy) + '\n' + \
                'Entropy max: ' + str(self._entropy_max) + '\n' + \
                'Average entropy max: ' + str(self._average_entropy_max) + '\n'

class object_profile:
    def __init__(self):
        # Initialize to empty
        self._id = -1  # object id
        self._associations = []  # associations to previous objects
        self._pose = [0,0,0,0,0,0,0,0,0]  # pose information
        self._probabilities = []  # class probabilities
        self._entropy = -1  # entropy
        
    def set(self, id, associations, pose, probabilities, entropy):
        # Set the values from the file
        self._id = id
        self._associations = associations
        self._pose = pose
        self._probabilities = probabilities
        self._entropy = entropy
        
    def __str__(self):
        return 'Id: ' + str(self._id) + '\n' + \
                'Associations: ' + str(self._associations) + '\n' + \
                'Pose: ' + str(self._pose) + '\n' + \
                'Probabilities: ' + str(self._probabilities) + '\n' + \
                'Entropy: ' + str(self._entropy) + '\n'

class object_list:
    def __init__(self):
        self._object_profiles = []
    
    def set(self, filename):
        # Read the filename and set the list of object profiles
         with open(filename) as f:
            # Read the lines
            lines = f.readlines()
            line_counter = -1
            while line_counter < len(lines):
                # Increment the line counter
                line_counter += 1
                # Get the words in the associations line
                if line_counter >= len(lines):
                    break
                # Split the line
                words = lines[line_counter].split()
                if len(words) > 0:
                    if words[0] == 'object':
                        # word[1] = id
                        id = int(words[1])
                        # Get the next line
                        line_counter += 1
                        # Get the words in the associations line
                        if line_counter >= len(lines):
                            break
                        assoc_words = lines[line_counter].split()
                        associations = []
                        if len(assoc_words) > 0:
                            # If the first word is "null" then ignore the associations
                            if assoc_words[0] != 'null':
                                # Read the entries and store in associations
                                for w in assoc_words:
                                    associations.append(int(w))
                        # Get the next line
                        line_counter += 1
                        # Get the pose line
                        if line_counter >= len(lines):
                            break
                        pose = [0,0,0,0,0,0,0,0,0]
                        pose_words = lines[line_counter].split()
                        if len(pose_words) == 9:
                            for i in range(0,8):
                                pose[i] = pose_words[i]
                        # Get the next line
                        line_counter += 1
                        if line_counter >= len(lines):
                            break
                        # Get the classification probabilities line
                        prob_words = lines[line_counter].split()
                        probabilities = []
                        if len(prob_words) > 0:
                            for i in range(0, len(prob_words), 2):
                                # Class name
                                class_name = prob_words[i]
                                # Remove the backslasj
                                class_name = class_name[:-1]
                                # Probability
                                prob = 0
                                if (i+1) < len(prob_words):
                                    prob = float(prob_words[i+1])
                                # Add to the list
                                probabilities.append([class_name, prob])
                        # Get the next line
                        line_counter += 1
                        if line_counter >= len(lines):
                            break
                        # Get the entropy line
                        ent_words = lines[line_counter].split()
                        entropy = -1
                        if len(ent_words) > 0:
                            entropy = ent_words[0]
                        else:
                            break
                        # Set the object profile
                        of = object_profile()
                        of.set(id, associations, pose, probabilities, entropy)
                        self._object_profiles.append(of)
    
    def print_all(self):
        for o in self._object_profiles:
            print o

def get_RGB_colors(N):
    #HSV_tuples = [(x*1.0/N, 0.8, 0.8) for x in range(N)]
    #RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
    cmap = plt.cm.brg
    RGB_tuples = cmap(np.linspace(0, 1, N))
    return RGB_tuples

def minimum_local_maximum(plan_set, class_type):
    object_probs = []
    if len(plan_set) > 0:
        o_set = plan_set[0]._object_profiles
        object_probs = [[0] for o in o_set]
    
    # Iterate through each view
    for ol in plan_set:
        # Iterate through each object at this view
        for oo in range(len(ol._object_profiles)):
            # Get the local minimum for this object
            if not class_type:
                object_probs[oo].append(ol._object_profiles[oo]._probabilities[0][1])
            else:
                # Search for the class type
                for c in ol._object_profiles[oo]._probabilities:
                    if c[0] == class_type:
                        object_probs[oo].append(c[1])
                        break
    
    # For each object find the local maximum
    local_maxs = []
    for o in object_probs:
        l_max = find_local_maximum(o)
        if len(l_max) > 0:
            local_maxs.append(l_max)
    # The average local maximum is the minimum of all
    return min(local_maxs)[0]*_SCALE_MAX_CONFIDENCE

def find_local_maximum(vec):
    # Find the local minimum in a vector of numbers
    # Start from the first element and continue while the next element is less
    # If an element is greater than, then found the local minimum
    local_max = []
    if len(vec) > 0:
        local_max.append(vec[0])
        for v in vec[1:]:
            if v > local_max[0]:
                local_max[0] = v
            else:
                # Found the local min so exit
                break
    return local_max

def flatten_end(vals):
    # Keep the vector monotonic (find if chould be increasing or decreasing)
    result = []
    if len(vals) > 1:
        best_val = vals[0]
        next_val = vals[1]
        # If the second element is less than the first then keep list decreasing
        decreasing = True
        if next_val > best_val:
            decreasing = False
        result.append(best_val)
        for v in vals[1:]:
            if decreasing:
                if v < best_val:
                    best_val = v
            else:
                if v > best_val:
                    best_val = v
            result.append(best_val)
    return result

def find_ignore_objects_file(dir):
    fname = dir + _IGNORE_OBJECTS_FILENAME
    ignore_objects = []
    number_classes = -1
    number_objects = -1
    if os.path.isfile(fname):
        with open(fname) as f:
            # Read the lines
            lines = f.readlines()
            if len(lines) == 2:
                indices = lines[0].split()
                for i in indices:
                    ignore_objects.append(int(i))
                nums = lines[1].split()
                if len(nums) == 2:
                    number_classes = nums[0]
                    number_objects = nums[1]
            #for line in lines:
            #    words = line.split()
            #    for word in words:
            #        ignore_objects.append(int(word))
    return (ignore_objects, number_classes, number_objects)

def save_ignore_objects_file(dir, number_classes, number_objects):
    # Search for the probability files
    fname = dir + _OBJECT_PROBABILITIES_FILENAME
    # If this is a valid file in the directory
    if os.path.isfile(fname):
        print 'directory ' + dir
        # Extract the probabilities and save the object_ignore file
        res, profiles = read_directory(dir, True, [], -1, -1)
        entropy_array, number_segments_array = get_object_entropies(profiles)
        ignore_objects_list = []
        for i in range(len(entropy_array)):
            # Count the number of views by the number of changes in the entropy value
            num_views = 0
            previous_val = entropy_array[i][0]
            for e in entropy_array[i][1:]:
                if not e == previous_val:
                    num_views += 1
                previous_val = e
            # Invalid if less than _MIN_VIEWS_FOR_VALID_OBJECT
            if num_views < _MIN_VIEWS_FOR_VALID_OBJECT:
                ignore_objects_list.append(i)
        # Write the file
        fname = dir + _IGNORE_OBJECTS_FILENAME
        f = open(fname, 'w')
        if len(ignore_objects_list) > 0:
            for i in ignore_objects_list:
                f.write(str(i))
                f.write(' ')
            f.write('\n')
            f.write(str(number_classes))
            f.write(' ')
            f.write(str(number_objects))
        print 'saved file!'
    else:
        # Go through each directory and search them instead
        subdirs = [x for x in  os.listdir(dir)]
        for s in subdirs:
            # Search directory
            sub = dir + s + '/'
            save_ignore_objects_file(sub, number_classes, number_objects)
            
def save_min_views_file(means, stds):
    # Write the minimum views to a file that can be quickly put into latex
    # e.g. 'NAME & 7.8 (2.3) & 9.2 (1.6) & 10.0 (0.0) & 9.4 (1.3) & 9.5 (1.5) & 9.3 (1.5) & 9.2 (1.8) &  8.6 (1.9) &  9.1 (1.5) \\'
    
    dec_places = 1
    # Go in reverse order of the input
    means.reverse()
    stds.reverse()
    f = open(_LATEX_TABLE_FILENAME, 'w')
    f.write('set')
    for i in range(len(means)):
        s = ' & ' + str(round(means[i],dec_places)) + ' (' + \
            str(round(stds[i],dec_places)) + ')'
        f.write(s)
    f.write(' \\')
    f.write('\\')

def read_directory(dir, extract_objects, ignore_objects, number_classes, number_objects):
    results = []
    object_profiles = []
    
    number_classes = int(number_classes)
    number_objects = int(number_objects)
    
    unknown_prob = 1.0
    if number_classes > 0:
        unknown_prob = 1.0 / number_classes
    
    # Read the object profiles
    remove_entropies = [[]]
    remove_num_objects = [[]]
    if extract_objects == True:
        p_ix = 0
        while True:
            prof_filename = dir + _OBJECT_FILE_PREFIX + str(p_ix) + '.txt'
            #print prof_filename
            if os.path.isfile(prof_filename):
                ol = object_list()
                ol.set(prof_filename)
                object_profiles.append(ol)
                #ol.print_all()
            else:
                break
            p_ix += 1
            
        # If ignoring objects then remove their entropy contribution
        # and their profiles
        if len(ignore_objects) > 0:
            remove_entropies, remove_num_objects = get_object_entropies(object_profiles)
        
    # Read the results file
    res_filename = dir + _RESULTS_FILE
    with open(res_filename) as f:
        view_count = 0
        for line in f:
            rp = results_profile()
            data = line.split()
            rp.set(data)
            
            if len(remove_entropies) > 0 and len(remove_num_objects) > 0 and len(ignore_objects) > 0:
                for o in ignore_objects:
                    ent = remove_entropies[o][view_count]
                    n = remove_num_objects[o][view_count]
                    if n > 0 and ent >= 0:
                        num_remaining = len(object_profiles[-1]._object_profiles) - n
                        
                        rp._entropy -= ent
                        av = rp._average_entropy
                        av *= len(object_profiles[-1]._object_profiles)
                        av -= ent
                        av /= num_remaining
                        rp._average_entropy = av
                        rp._entropy_max -= ent
                        if number_objects > 0 and number_classes > 0 and num_remaining < number_objects:
                            num_extra_objects = number_objects - num_remaining
                            extra_entropy = num_extra_objects * number_classes * unknown_prob * log(unknown_prob)
                            rp._entropy_max += extra_entropy
                        av = rp._average_entropy_max
                        av *= len(object_profiles[-1]._object_profiles)
                        av -= ent
                        if number_objects > 0 and number_classes > 0 and num_remaining < number_objects:
                            num_extra_objects = number_objects - num_remaining
                            extra_entropy = num_extra_objects * number_classes * unknown_prob * log(unknown_prob)
                            rp._average_entropy_max += extra_entropy
                        av /= num_remaining
                        rp._average_entropy_max = av
                        
                        #print str(rp._entropy) + ' ' + str(rp._entropy_max) + ' ' + \
                        #      str(rp._average_entropy) + ' ' + \
                        #      str(rp._average_entropy_max)

            results.append(rp)
            view_count += 1
            
    # Remove the profiles
    if len(ignore_objects) > 0:
        num_original = len(object_profiles[-1]._object_profiles)
        #print 'Removing objects [' + ' '.join(map(str, ignore_objects)) + ']'
        ignore_objects.sort()  # sort
        ignore_objects = ignore_objects[::-1]  # reverse
        for o in ignore_objects:
            object_profiles[-1]._object_profiles.pop(o)  # remove
        #print 'Number of objects resized from ' + str(num_original) + \
        #      ' to ' + str(len(object_profiles[-1]._object_profiles))
        
    return (results, object_profiles)

def read_plan_set_directory(dir, extract_objects, ignore_invalid_objects):
    # Get all the subdirectories (plans)
    planner_names = [x for x in  os.listdir(dir)]
    
    # Within each subdirectory read the results file
    planner_results = []
    for p in planner_names:
        subdir = dir + p + '/'
        # Find an ignore objects file in the directory
        ignore_objects = []
        number_classes = -1
        number_objects = -1
        if ignore_invalid_objects:
            ignore_objects, number_classes, number_objects = find_ignore_objects_file(subdir)
            #if len(ignore_objects) > 0:
                #print 'Found ignore objects file!'
                #print 'Ignore objects: ' + ' '.join(map(str, ignore_objects))
        
        # Read the results
        res, prof = read_directory(subdir, extract_objects, ignore_objects, number_classes, number_objects)
        #print 'Size of res ' + str(len(res))
        # Check that each entropy value is valid
        is_valid = True
        for r in res:
            if r._entropy < 0 or r._average_entropy < 0 or \
               r._entropy_max < 0 or r._average_entropy_max < 0:
                is_valid = False
        if is_valid == True:
            planner_results.append([p,res, prof])
    return planner_results

def read_simulation_set_directory(dir, ignore_invalid_objects):
    # Get all the subdirectories (different start indices)
    index_names = [x for x in  os.listdir(dir)]
    print 'Num simulations in dataset ' + str(len(index_names))
    
    # Within each subdirectory read all the plan set directories
    simulation_results = []
    for i in index_names:
        subdir = dir + i + '/'
        # Read the plan set
        pset = read_plan_set_directory(subdir, True, ignore_invalid_objects)
        # Add to the list
        for p in pset:
            simulation_results.append([i,p[0],p[1],p[2]])
    return simulation_results

def get_object_entropies(profiles):
    # Get the number of objects to view
    num_objects = 0
    num_views = 0
    entropy_results_array = [[]]
    number_segments_array = [[]]
    if len(profiles) > 0:
        num_objects = len(profiles[-1]._object_profiles)
        num_views = len(profiles)
    else:
        return (entropy_results_array, number_segments_array)
    # Add 1 to the number of views
    num_views += 1
    
    if num_objects > 0 and num_views > 0:
        # For each object
        object_count = -1
        entropy_results_array = []
        number_segments_array = []
        for o in range(len(profiles[-1]._object_profiles)):
            object_count += 1
            entropy_vec = []
            entropy_vec.append(float(profiles[-1]._object_profiles[o]._entropy))
            num_segs = []
            num_segs.append(1)
            
            # Search for its associations in the previous view
            view_num = -2
            view_count = 1
            assocs = profiles[-1]._object_profiles[o]._associations
            while len(assocs) > 0 and view_count < num_views:
                combined_entropy_result = 0
                for a in assocs:
                    # The entropy
                    combined_entropy_result += float(profiles[view_num]._object_profiles[a]._entropy)
                # Add to the list
                #class_results_array[object_count].append(combined_class_result)
                entropy_vec.append(combined_entropy_result)
                num_segs.append(len(assocs))
                # Next iteration
                new_assocs = []
                for a in assocs:
                    for p in profiles[view_num]._object_profiles[a]._associations:
                        new_assocs.append(p)
                assocs = new_assocs
                view_num -= 1
                view_count += 1
                
            # If the views do not go all the way to the beginning then fill with negative entropy values
            max_view_ix = num_views
            if view_count != max_view_ix:
                while view_count < num_views:
                    # Entropy value
                    entropy_vec.append(-1)
                    num_segs.append(0)
                    view_count += 1
            
            # Reverse the arrays
            entropy_vec = entropy_vec[::-1]
            num_segs = num_segs[::-1]
            # Add to the arrays for all the objects
            entropy_results_array.append(entropy_vec)
            number_segments_array.append(num_segs)
            
        # Format the output in the correct order
        #print entropy_results_array
        #print number_segments_array
        
    # Return
    return (entropy_results_array, number_segments_array)

def visualize_entropy_results(results, profiles, ignore_objects):
    # Plot the entropies
    x = range(len(results))
    entropies = [r._entropy for r in results]
    average_entropies = [r._average_entropy for r in results]
    entropies_max = [r._entropy_max for r in results]
    average_entropies_max = [r._average_entropy_max for r in results]
    
    f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex='col')
    ax1.plot(x, entropies)
    ax1.set_title(r'Entropy')
    ax3.plot(x, average_entropies)
    ax3.set_title(r'Average Entropy')
    ax2.plot(x, entropies_max)
    ax2.set_title(r'Max Entropy')
    ax4.plot(x, average_entropies_max)
    ax4.set_title(r'Average Max Entropy')
    
def visualize_object_profiles(profiles, training_directory, class_type):
    # Get the class names
    class_names = [x for x in  os.listdir(training_directory)]
    class_ix = np.arange(len(class_names))
    unknown_prob = 1.0 / len(class_ix)
    
    class_names_labels = class_names
    for c in range(len(class_names_labels)):
        class_names_labels[c] = class_names_labels[c].replace("_", " ")
    
    # Get the number of objects to view
    num_objects = 0
    num_views = 0
    if len(profiles) > 0:
        num_objects = len(profiles[-1]._object_profiles)
        print 'Number of objects ' + str(num_objects)
        # Count the number of elements in ignore_objects that are less than num_objects
        #if len(ignore_objects) > 0:
        #    print 'Removing objects [' + ' '.join(map(str, ignore_objects)) + ']'
        #    num_subtract = sum(i < num_objects for i in ignore_objects)
        #    num_objects -= num_subtract
        #    print 'Number of objects resized to ' + str(num_objects)
        num_views = len(profiles)
    # Add 1 to the number of views
    num_views += 1
    
    if num_objects > 0 and num_views > 0:
        width = 1.0
        RGB = get_RGB_colors(len(class_names))
        # Set a subfigure for each object and for each view
        f2, axarr = plt.subplots(num_views, num_objects, sharex=True)
        if num_objects == 1:
            narr = []
            for ax in axarr:
                narr.append([ax])
            axarr = np.array(narr)
        # For each object
        object_count = -1
        class_results_array = [[] for x in range(num_objects)]
        for o in range(len(profiles[-1]._object_profiles)):
            #if o not in ignore_objects:
            object_count += 1
            prob_vec = [0 for x in range(len(class_names))]
            for c in range(len(class_names)):
                for p in profiles[-1]._object_profiles[o]._probabilities:
                    if p[0] == class_names[c]:
                        prob_vec[c] = p[1]
            barlist = axarr[0][object_count].bar(class_ix, prob_vec, width, color=[0,1,0])
            axarr[0][object_count].set_xlim(0,len(class_names))
            axarr[0][object_count].set_ylim(0,1)
            axarr[0][object_count].set_xticks(class_ix+width/2)
            xtickNames = axarr[0][object_count].set_xticklabels(class_names_labels)
            axarr[0][object_count].yaxis.set_ticks([0,1])
            for b in range(len(barlist)):
                barlist[b].set_color(RGB[b])
                barlist[b].set_edgecolor('black')
            
            # Add the element to the class_results_array
            found_prob = False
            if num_objects == 1 and class_type:
                for p in profiles[-1]._object_profiles[o]._probabilities:
                    if p[0] == class_type:
                        class_results_array[object_count].append(p[1])
                        found_prob = True
                if found_prob == False:
                    class_results_array[object_count].append(unknown_prob)
                    found_prob = True
            if found_prob == False:
                class_results_array[object_count].append(profiles[-1]._object_profiles[o]._probabilities[0][1])
            
            # Search for its associations in the previous view
            view_num = -2
            view_count = 1
            assocs = profiles[-1]._object_profiles[o]._associations
            while len(assocs) > 0 and view_count < num_views:
                prob_vec = [0 for x in range(len(class_names))]
                combined_class_result = 0
                for a in assocs:
                    # The probability vector
                    for c in range(len(class_names)):
                        for p in profiles[view_num]._object_profiles[a]._probabilities:
                            if p[0] == class_names[c]:
                                prob_vec[c] += p[1]
                    # The max probability
                    found_prob = False
                    if num_objects == 1 and class_type:
                        for p in profiles[view_num]._object_profiles[a]._probabilities:
                            if p[0] == class_type:
                                combined_class_result += p[1]
                                found_prob = True
                        if found_prob == False:
                            combined_class_result += unknown_prob
                            found_prob = True
                    if found_prob == False:
                        combined_class_result += profiles[view_num]._object_profiles[a]._probabilities[0][1]
                
                # Normalize the probability vector
                norm_const = sum(prob_vec)
                prob_vec[:] = [x/norm_const for x in prob_vec]
                # Average the combined class result
                combined_class_result /= len(assocs)
                # Add to the list
                class_results_array[object_count].append(combined_class_result)
                
                # Plot
                barlist = axarr[view_count][object_count].bar(class_ix, prob_vec, width, color='blue')
                axarr[view_count][object_count].set_xlim(0,len(class_names))
                axarr[view_count][object_count].set_ylim(0,1)
                axarr[view_count][object_count].set_xticks(class_ix+width/2)
                xtickNames = axarr[view_count][object_count].set_xticklabels(class_names_labels)
                axarr[view_count][object_count].yaxis.set_ticks([0,1])
                for b in range(len(barlist)):
                    barlist[b].set_color(RGB[b])
                    barlist[b].set_edgecolor('black')
                # Next iteration
                new_assocs = []
                for a in assocs:
                    for p in profiles[view_num]._object_profiles[a]._associations:
                        new_assocs.append(p)
                assocs = new_assocs
                view_num -= 1
                view_count += 1
                
            # If the views do not go all the way to the beginning then fill with even probabilities
            max_view_ix = num_views
            if view_count != max_view_ix:
                while view_count < num_views:
                    even_val = 1/len(class_ix)
                    # Class results
                    class_results_array[object_count].append(even_val)
                    #Plot
                    prob_vec = [even_val for x in range(len(class_ix))]
                    barlist = axarr[view_count][object_count].bar(class_ix, prob_vec, width, color='blue')
                    axarr[view_count][object_count].set_xlim(0,len(class_names))
                    axarr[view_count][object_count].set_ylim(0,1)
                    axarr[view_count][object_count].set_xticks(class_ix+width/2)
                    axarr[view_count][object_count].yaxis.set_ticks([0,1])
                    for b in range(len(barlist)):
                        barlist[b].set_color(RGB[b])
                        barlist[b].set_edgecolor('black')
                    view_count += 1
        
        # Rotate the x labels
        for x in range(num_objects):
            #xtickNames = axarr[view_count][object_count].set_xticklabels(class_names)
            plt.setp(axarr[-1][x].xaxis.get_majorticklabels(), rotation=90)
            
        # Plot the classification result for each object
        print class_results_array
        f1, ax1 = plt.subplots()
        x = np.arange(num_views+1)
        for c in range(len(class_results_array)):
            # Add lowest probability to the beginning
            p_vec = np.append(np.array(class_results_array[c]), unknown_prob)
            # Reverse
            p_vec = p_vec[::-1]
            # Plot
            l = r'object ' + str(c)
            ax1.plot(x, p_vec, label=l)
            
        # Take the mean of the probabilities for the objects to have a single
        # value for each view
        class_results_array = np.array(class_results_array)
        class_results_array = np.mean(class_results_array, axis=0)
        # Add lowest probability to the beginning
        class_results_array = np.append(class_results_array, unknown_prob)
        # Reverse the array of probabilities
        class_results_array = class_results_array[::-1]
        # Plot the mean
        x_interp = np.linspace(x[0], x[-1], len(x)*2)
        tck = interpolate.splrep(x, class_results_array, s=0.15)
        y_interp = interpolate.splev(x_interp, tck, der=0)
        y_interp[0] = class_results_array[0]
        ax1.plot(x_interp, y_interp, color='k', linewidth=2, label=r'mean') 
        handles, labels = ax1.get_legend_handles_labels()
        ax1.legend(handles, labels)        
    
def visualize_planners(planner_results, ignore_plans):
    Xlim = 7
    s_interp = 0.001
    do_interp = False
    do_flatten = False
    # Plot the entropies of each planner
    if len(planner_results) > 0:
        # Get the maximum length of all the planners
        max_length = 0
        for p in planner_results:
            if len(p[1]) > max_length:
                max_length = len(p[1])
        # X axis
        x = range(max_length)
        
        # Get the planner names in order 
        planner_names = [p[0] for p in planner_results]
    
        # Order planner names in the order of the _PLAN_ORDER
        ordered_planner_names = ['' for pn in planner_names]
        unused_planner_names = []
        num_added = 0
        for i in range(len(planner_names)):
            if planner_names[i] in ignore_plans:
                found = True
            else:
                found = False
                for j in range(len(_PLAN_ORDER)):
                    if planner_names[i] == _PLAN_ORDER[j]:
                        found = True
                        ordered_planner_names[num_added] = planner_names[i]
                        num_added += 1
                        break
            # If this plan was not found then add to the unused list
            if found == False:
                unused_planner_names.append(planner_names[i])
        # If some plans were not added then add to the end
        if len(unused_planner_names) > 0:
            for u in unused_planner_names:
                ordered_planner_names[num_added] = u
                num_added += 1
        planner_names = ordered_planner_names
        # Remove empty strings
        planner_names = filter(None, planner_names)
        print planner_names
        
        planner_results_labels = []
        for p in range(len(planner_results)):
            planner_results_labels.append(planner_results[p][0].replace("_", " "))
        
        # Get the colors
        # First element (worst method) is dashed black
        # Last element (best method) is solid black
        RGB = get_RGB_colors(len(planner_names))
        line_styles = ['-' for p in planner_names]
##        if len(planner_names) > 2:
##            worst_col = np.array([0.2,0.2,0.2,1])  # grey
##            best_col = np.array([0,0,0,1])   # black
##            RGB = get_RGB_colors(len(planner_names)-2)
##            RGB = np.vstack((worst_col,RGB))
##            RGB = np.vstack((RGB,best_col))
##            line_styles = ['--']
##            for i in planner_names[1]:
##                line_styles.append('-')
        
        # Plot each planner
        f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex='col')
        all_entropies = []
        for i in range(len(planner_names)):
            for j in range(len(planner_results)):
                if planner_names[i] == planner_results[j][0]:
                    entropies = [r._entropy for r in planner_results[j][1]]
                    average_entropies = [r._average_entropy for r in planner_results[j][1]]
                    entropies_max = [r._entropy_max for r in planner_results[j][1]]
                    average_entropies_max = [r._average_entropy_max for r in planner_results[j][1]]
                        
                    #entropies = flatten_end(entropies)
                    #average_entropies = flatten_end(average_entropies)
                    #entropies_max = flatten_end(entropies_max)
                    #average_entropies_max = flatten_end(average_entropies_max)
                    
                    # Fill the vectors if they are short on views
                    if len(entropies) != max_length:
                        while len(entropies) < max_length:
                            entropies.append(entropies[-1])
                    if len(average_entropies) != max_length:
                        while len(average_entropies) < max_length:
                            average_entropies.append(average_entropies[-1])
                    if len(entropies_max) != max_length:
                        while len(entropies_max) < max_length:
                            entropies_max.append(entropies_max[-1])
                    if len(average_entropies_max) != max_length:
                        while len(average_entropies_max) < max_length:
                            average_entropies_max.append(average_entropies_max[-1])
                            
                    # Store in the vector
                    all_entropies.append([planner_results_labels[j],average_entropies_max])
                            
                    x_interp = np.linspace(x[0], x[-1], len(x)*2)
                    tck = interpolate.splrep(x, entropies, s=s_interp)
                    y_interp = interpolate.splev(x_interp, tck, der=0)
                    #ax1.plot(x_interp, y_interp, color=RGB[i], linestyle=line_styles[i], label=planner_results[j][0])  
                    ax1.plot(x, entropies, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=planner_results_labels[j])       
                    ax3.plot(x, average_entropies, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=planner_results_labels[j])           
                    ax2.plot(x, entropies_max, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=planner_results_labels[j])            
                    ax4.plot(x, average_entropies_max, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=planner_results_labels[j])
            
        # Set the axis labels
        ax1.set_title(r'Entropy')
        #box1 = ax1.get_position()
        #ax1.set_position([box1.x0, box1.y0, box1.width * 0.5, box1.height])
        handles, labels = ax1.get_legend_handles_labels()
        #ax1.legend(handles, labels)
        ax3.set_title(r'Average Entropy')
        #handles, labels = ax3.get_legend_handles_labels()
        #ax3.legend(handles, labels)
        ax2.set_title(r'Max Entropy')
        #handles, labels = ax2.get_legend_handles_labels()
        #ax2.legend(handles, labels)
        ax4.set_title(r'Average Max Entropy')
        #handles, labels = ax4.get_legend_handles_labels()
        #ax4.legend(handles, labels)
        plt.subplots_adjust(right=0.8)
        plt.figlegend(handles, labels, loc = 'right', ncol=1, \
                      labelspacing=1.5, fontsize=12, bbox_to_anchor=(0.975,0.5))
                    
        # Plot an interpolated line of the entropy
        f_interp, ax_interp = plt.subplots()
        cut_off = Xlim + 1
        for i in range(len(all_entropies)):
            e = np.array(all_entropies[i][1])
            e = e[np.isfinite(e)]
            x = range(len(e))
            if (cut_off > 0 and cut_off < len(e)):
                e = e[:cut_off]
                x = x[:cut_off]
            if do_flatten:
                e = flatten_end(e)
            x_interp = np.linspace(x[0], x[-1], len(x)*2)
            tck = interpolate.splrep(x, e, s=s_interp)
            y_interp = interpolate.splev(x_interp, tck, der=0)
            if do_interp:
                ax_interp.plot(x_interp, y_interp, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=all_entropies[i][0])
            else:
                ax_interp.plot(x, e, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=all_entropies[i][0])
        ax_interp.set_xlim([0,Xlim])
        handles, labels = ax_interp.get_legend_handles_labels()
##        plt.figlegend(handles, labels, loc = 'right', ncol=1, \
##                      labelspacing=1.5, fontsize=12, bbox_to_anchor=(0.975,0.5))
        plt.legend(handles, labels, fontsize=16)
        plt.xlabel(r'Number of views', fontsize=20)
        plt.ylabel(r'Entropy', fontsize=20)
        plt.tick_params(axis='both', which='major', labelsize=20)
          
    # Plot the average recognition probability for each planner
    f_av_p, ax_av_p1 = plt.subplots(1, 1)
    for i in range(len(planner_names)):
        for j in range(len(planner_results)):
            if planner_names[i] == planner_results[j][0]:
                # Object profiles
                av_prob = average_object_probabilities(planner_results[j][2])
                # Plot the mean
                a = av_prob
                x = range(len(av_prob))
                if (cut_off > 0 and cut_off < len(a)):
                    a = a[:cut_off]
                    x = x[:cut_off]
                if do_flatten:
                    a = flatten_end(a)
                # Interpolate a smooth line
                x_interp = np.linspace(x[0], x[-1], len(x)*2)
                tck = interpolate.splrep(x, a, s=s_interp)
                y_interp = interpolate.splev(x_interp, tck, der=0)
                y_interp[0] = av_prob[0]
                if do_interp:
                    ax_av_p1.plot(x_interp, y_interp, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=all_entropies[i][0])
                else:
                    ax_av_p1.plot(x, a, linewidth=2, color=RGB[i], linestyle=line_styles[i], label=all_entropies[i][0])
    ax_av_p1.set_xlim([0,Xlim])
    #ax_av_p2.set_xlim([0,7])
    plt.xlabel(r'Number of views', fontsize=20)
    plt.ylabel(r'Mean Max Probability', fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=20)
    handles, labels = ax_av_p1.get_legend_handles_labels()
    ax_av_p1.legend(handles, labels, loc='lower right', fontsize=16)
        
def average_object_probabilities(profiles):
    # Get the number of objects to view
    num_objects = 0
    num_views = 0
    if len(profiles) > 0:
        num_objects = len(profiles[-1]._object_profiles)
        print 'Number of objects ' + str(num_objects)
        num_views = len(profiles)
    # Add 1 to the number of views
    num_views += 1
    
    if num_objects > 0 and num_views > 0:
        # For each object
        object_count = -1
        class_results_array = [[] for x in range(num_objects)]
        for o in range(len(profiles[-1]._object_profiles)):
            object_count += 1
            
            # Add the element to the class_results_array
            found_prob = False
            if num_objects == 1 and class_type:
                for p in profiles[-1]._object_profiles[o]._probabilities:
                    if p[0] == class_type:
                        class_results_array[object_count].append(p[1])
                        found_prob = True
                if found_prob == False:
                    class_results_array[object_count].append(unknown_prob)
                    found_prob = True
            if found_prob == False:
                class_results_array[object_count].append(profiles[-1]._object_profiles[o]._probabilities[0][1])
            
            # Search for its associations in the previous view
            view_num = -2
            view_count = 1
            assocs = profiles[-1]._object_profiles[o]._associations
            while len(assocs) > 0 and view_count < num_views:
                combined_class_result = 0
                for a in assocs:
                    # The max probability
                    found_prob = False
                    if num_objects == 1 and class_type:
                        for p in profiles[view_num]._object_profiles[a]._probabilities:
                            if p[0] == class_type:
                                combined_class_result += p[1]
                                found_prob = True
                        if found_prob == False:
                            combined_class_result += unknown_prob
                            found_prob = True
                    if found_prob == False:
                        combined_class_result += profiles[view_num]._object_profiles[a]._probabilities[0][1]
                
                # Average the combined class result
                combined_class_result /= len(assocs)
                # Add to the list
                class_results_array[object_count].append(combined_class_result)
                
                # Next iteration
                new_assocs = []
                for a in assocs:
                    for p in profiles[view_num]._object_profiles[a]._associations:
                        new_assocs.append(p)
                assocs = new_assocs
                view_num -= 1
                view_count += 1
                
            # If the views do not go all the way to the beginning then fill with even probabilities
            even_val = 1/15
            max_view_ix = num_views
            if view_count != max_view_ix:
                while view_count < num_views:
                    # Class results
                    class_results_array[object_count].append(even_val)
                    view_count += 1
            
        # Take the mean of the probabilities for the objects to have a single
        # value for each view
        class_results_array = np.array(class_results_array)
        class_results_array = np.mean(class_results_array, axis=0)
        # Reverse the array of probabilities
        class_results_array = class_results_array[::-1]
        # Return the mean
        return class_results_array
        
def visualize_minimum_views_one_object(simulation_results, confidence, class_type, ignore_plans, ignore_sets):
    print 'Visualizing results for set with one object'
    # Plot the number of views it takes to reach the desired level of confidence
    check_local_min = False
    if confidence == 0.0:
        check_local_min = True
    confidence_level = confidence
    check_class_type = True
    if not class_type:
        check_class_type = False
        
    # Replace ignore_sets element -1 with index_min_ent
    for i in range(len(ignore_sets)):
        if ignore_sets[i] == -1:
            ignore_sets[i] = _INDEX_MIN_ENT
    
    # Get the plan names
    index_names = []
    planner_names = []
    num_views_in_set = -1
    for s in simulation_results:
        if s[0] not in ignore_sets:
            index_names.append(s[0])
        planner_names.append(s[1])
        n = len(s[3])
        if n > num_views_in_set:
            num_views_in_set = n
    # Get the unique entries
    index_names = list(set(index_names))
    planner_names = list(set(planner_names))
    print 'Num views in set ' + str(n)
    # Order planner names in the order of the _PLAN_ORDER
    ordered_planner_names = ['' for x in planner_names]
    unused_planner_names = []
    num_added = 0
    for i in range(len(planner_names)):
        if planner_names[i] in ignore_plans:
            found = True
        else:
            found = False
            for j in range(len(_PLAN_ORDER)):
                if planner_names[i] == _PLAN_ORDER[j]:
                    found = True
                    ordered_planner_names[j] = planner_names[i]
                    num_added += 1
                    break
        # If this plan was not found then add to the unused list
        if found == False:
            unused_planner_names.append(planner_names[i])
    # If some plans were not added then add to the end
    if len(unused_planner_names) > 0:
        for u in unused_planner_names:
            ordered_planner_names[num_added] = u
            num_added += 1
    planner_names = ordered_planner_names
    # Remove empty strings
    planner_names = filter(None, planner_names)
    
    planner_names_labels = []
    for p in range(len(planner_names)):
        planner_names_labels.append(planner_names[p].replace("_", " "))
        
    if len(planner_names) == 0:
        print 'No planner names'
        return
    if len(index_names) == 0:
        print 'No sets'
        return
    
    # If check local min is set
    if check_local_min:
        found_local_min = False
        best_plan = planner_names[-1]
        while not found_local_min:
                for s in simulation_results:
                    if s[1] == best_plan:
                        confidence_level = minimum_local_maximum(s[3], class_type)
                        found_local_min = True
                        break
    print 'Confidence Level ' + str(confidence_level)
    
    # Get the number of views for each planner in each set
    planner_num_views = []
    for p in planner_names:
        #print 'Planner ' + p
        # For each index
        p_views = []
        for i in index_names:
            # Search through simulation_results to find the index and plan name
            v = 1
            for s in simulation_results:
                # If they match
                if s[0] == i and s[1] == p:
                    # For each object list (this is the view number)
                    #print ' - - - ' + s[0] + ' ' + s[1]
                    #print 'Set ' + str(i)
                    for ol in s[3]:
                        #print 'View'
                        success_classification = np.ones(len(ol._object_profiles))
                        # Iterate through each object at this view
                        #print 'Num objects ' + str(len(ol._object_profiles))
                        for oo in range(len(ol._object_profiles)):
                            #print ol._object_profiles[oo]._probabilities
                            # Count the number of views until all object probabilities
                            # are above confidence
                            #print ol._object_profiles[oo]._probabilities
                            # [0] is the highest scoring class, [1] is the actual probability
                            if ol._object_profiles[oo]._probabilities[0][1] >= confidence_level:
                                success_classification[oo] = 0
                                break
                        # v is the number of views that it has taken to reach
                        # confidence
                        if sum(success_classification) == 0:
                            break
                        v += 1
                    break
            # Add an entry to the planner for this index
            p_views.append(v)
        planner_num_views.append([p,p_views,np.mean(p_views),np.std(p_views)])
    
    # Print    
    print planner_num_views
    # Plot
    width = 0.75
    # Get the colors
    # First element (best method) is solid black
    # Last element (worst method) is dashed black
    RGB = get_RGB_colors(len(planner_num_views))
    hatches = [False for p in planner_num_views]
    if len(planner_num_views) > 2:
        worst_col = np.array([0.3,0.3,0.3,1])  # dark grey
        best_col = np.array([0.1,0.1,0.1,1])   # almost black
        RGB = get_RGB_colors(len(planner_num_views)-2)
        RGB = np.vstack((worst_col,RGB))
        RGB = np.vstack((RGB,best_col))
        hatches = [True]
        for i in range(len(planner_num_views)-1):
            hatches.append(False)
    means = [x[2] for x in planner_num_views]
    stds = [x[3] for x in planner_num_views]
    x = np.arange(len(planner_num_views))
    fig, ax = plt.subplots()
    barlist = ax.bar(x, means, width, color='r', yerr=stds, ecolor='k')
    ax.set_xlim(0,len(planner_names))
    ax.set_xticks(x+width/2)
    xtickNames = ax.set_xticklabels(planner_names_labels)
    for b in range(len(barlist)):
        barlist[b].set_color(RGB[b])
        barlist[b].set_edgecolor('black')
        if hatches[b]:
            barlist[b].set_hatch('//')
    # Rotate the x labels
    for b in range(len(planner_names)):
        #xtickNames = axarr[view_count][object_count].set_xticklabels(class_names)
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=75)
    # Adjust the plot size to fit the labels
    plt.tight_layout()
    # Add the values to the bars
    for ii, m in enumerate(means):
        height = m
        ax.text(x[ii]+width/2, height+0.2, '%0.1f'% m, fontsize=14, ha='center', va='bottom', backgroundcolor='w')

def visualize_minimum_views(simulation_results, confidence, class_type, ignore_plans, ignore_sets):
    # Plot the number of views it takes to reach the desired level of confidence
    check_local_min = False
    if confidence == 0.0:
        check_local_min = True
    confidence_level = confidence
    check_class_type = True
    if not class_type:
        check_class_type = False
        
    # Replace ignore_sets element -1 with index_min_ent
    for i in range(len(ignore_sets)):
        if ignore_sets[i] == '-1':
            ignore_sets[i] = _INDEX_MIN_ENT
    
    # Get the plan names
    index_names = []
    planner_names = []
    num_views_in_set = -1
    for s in simulation_results:
        if s[0] not in ignore_sets:
            index_names.append(s[0])
        planner_names.append(s[1])
        n = len(s[3])
        if n > num_views_in_set:
            num_views_in_set = n
    # Get the unique entries
    index_names = list(set(index_names))
    planner_names = list(set(planner_names))
    print 'Num views in set ' + str(num_views_in_set)
    # Order planner names in the order of the _PLAN_ORDER
    ordered_planner_names = ['' for x in range(max(len(_PLAN_ORDER),len(planner_names)))]
    unused_planner_names = []
    num_added = 0
    for i in range(len(planner_names)):
        if planner_names[i] in ignore_plans:
            found = True
        else:
            found = False
            for j in range(len(_PLAN_ORDER)):
                if planner_names[i] == _PLAN_ORDER[j]:
                    found = True
                    ordered_planner_names[j] = planner_names[i]
                    num_added += 1
                    break
        # If this plan was not found then add to the unused list
        if found == False:
            unused_planner_names.append(planner_names[i])
    # If some plans were not added then add to the end
    if len(unused_planner_names) > 0:
        for u in unused_planner_names:
            ordered_planner_names[num_added] = u
            num_added += 1
    planner_names = ordered_planner_names
    # Remove empty strings
    planner_names = filter(None, planner_names)
    print planner_names
    
    if len(planner_names) == 0:
        print 'No planner names'
        return
    if len(index_names) == 0:
        print 'No sets'
        return
    
    # If check local min is set
    if check_local_min:
        found_local_min = False
        best_plan = planner_names[-1]
        while not found_local_min:
                for s in simulation_results:
                    if s[1] == best_plan:
                        confidence_level = minimum_local_maximum(s[3], class_type)
                        found_local_min = True
                        break
    print 'Confidence Level ' + str(confidence_level)
    
    # Get the number of views for each planner in each set
    planner_num_views = []
    planner_num_views_separate = []
    for p in planner_names:
        #print 'Planner ' + p
        # For each index
        p_views = []
        p_views_separate = []
        for i in index_names:
            # Search through simulation_results to find the index and plan name
            v = []
            for s in simulation_results:
                # If they match then this is a planner and a set
                if s[0] == i and s[1] == p:
                    # For each object list (this is the view number)
                    #print ' - - - ' + s[0] + ' ' + s[1]
                    #print 'Set ' + str(i)
                    # Extract the objects in the first view
                    # Extract the probabilities for each object separately
                    all_views = s[3]
                    # For each object
                    final_objects = s[3][-1]._object_profiles
                    object_probs = [[] for x in range(len(final_objects))]
                    for o in range(len(final_objects)):
                        #print 'object'
                        # Append the last vector
                        object_probs[o].append(final_objects[o]._probabilities)
                        # For its associations
                        view_num = -2
                        assocs = final_objects[o]._associations
                        iter = 1
                        while len(assocs) > 0 and iter < num_views_in_set:
                            p_sum = [[x[0], 0] for x in object_probs[o][iter-1]]
                            next_assocs = []
                            for a in assocs:
                                # Get the probabiliies in the previous associations
                                p_vec = s[3][view_num]._object_profiles[a]._probabilities
                                unmatched_classes = []
                                for pv in p_vec:
                                    found_in_vec = False
                                    for ps in range(len(p_sum)):
                                        if pv[0] == p_sum[ps][0]:
                                            p_sum[ps][1] += pv[1]
                                            found_in_vec = True
                                            break
                                    if found_in_vec == False:
                                        unmatched_classes.append(pv)
                                if len(unmatched_classes) > 0:
                                    p_vec.extend(unmatched_classes)
                                if len(s[3][view_num]._object_profiles[a]._associations) > 0:
                                    next_assocs.extend(s[3][view_num]._object_profiles[a]._associations)
                            # Normalise the probabilities
                            norm_const = sum(x[1] for x in p_vec)
                            for pv in range(len(p_vec)):
                                p_vec[pv][1] /= norm_const
                            # Sort by the largest probability
                            p_vec = sorted(p_vec,key=lambda x: x[1])
                            # Reverse
                            p_vec = p_vec[::-1]
                            # Append the vector
                            object_probs[o].append(p_vec)
                            iter += 1
                            view_num -= 1
                            assocs = list(set(next_assocs))
                            
                        # If this object does not reach the beginning
                        while len(object_probs[o]) < num_views_in_set:
                            object_probs[o].append([['null_class',0]])

                    # Go through the object probabilities from the back to find
                    # which view the object is classified above the thershold
                    object_min_views = [(num_views_in_set+1) for o in object_probs]
                    for o in range(len(object_probs)):
                        # Reverse the elements
                        rev_vec = object_probs[o][::-1]
                        for vv in range(len(rev_vec)):
                            if rev_vec[vv][0][1] >= confidence_level:
                                object_min_views[o] = vv+1
                                break
                    v.extend(object_min_views)
            # Add an entry to the planner for this index
            if len(v) > 0:
                p_views.append(max(v))  # do it this way for considering each set separate and only getting the max
                p_views_separate.extend(v)  # do it this way if summing all objects individually
        planner_num_views.append([p,p_views,np.mean(p_views),np.std(p_views)])
        planner_num_views_separate.append([p,p_views_separate,np.mean(p_views_separate),np.std(p_views_separate)])
    
    # Print
    for p in planner_num_views:
        print p
    # Plot
    width = 0.75
    # Replace underscores with whitespace
    for p in range(len(planner_names)):
        planner_names[p] = planner_names[p].replace("_", " ")
    # Get the colors
    # First element (best method) is solid black
    # Last element (worst method) is dashed black
    RGB = get_RGB_colors(len(planner_num_views))
    hatches = [False for p in planner_num_views]
    if len(planner_num_views) > 2:
        worst_col = np.array([0.3,0.3,0.3,1])  # dark grey
        best_col = np.array([0.1,0.1,0.1,1])   # almost black
        RGB = get_RGB_colors(len(planner_num_views)-2)
        RGB = np.vstack((worst_col,RGB))
        RGB = np.vstack((RGB,best_col))
        hatches = [True]
        for i in range(len(planner_num_views)-1):
            hatches.append(False)
    means = [x[2] for x in planner_num_views]
    stds = [x[3] for x in planner_num_views]
    x = np.arange(len(planner_num_views))
    fig, ax = plt.subplots()
    barlist = ax.bar(x, means, width, color='r', yerr=stds, ecolor='k')
    ax.set_xlim(0,len(planner_names))
    ax.set_xticks(x+width/2)
    xtickNames = ax.set_xticklabels(planner_names)
    for b in range(len(barlist)):
        barlist[b].set_color(RGB[b])
        barlist[b].set_edgecolor('black')
        if hatches[b]:
            barlist[b].set_hatch('//')
    # Rotate the x labels
    for b in range(len(planner_names)):
        #xtickNames = axarr[view_count][object_count].set_xticklabels(class_names)
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=90, fontsize=20)
    # Adjust the plot size to fit the labels
    plt.tight_layout()
    # Add the values to the bars
    for ii, m in enumerate(means):
        height = m
        ax.text(x[ii]+width/2, height+0.2, '%0.1f'% m, fontsize=20, ha='center', va='bottom', backgroundcolor='w')
    plt.ylabel(r'Number of views', fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=20)
    plt.title(r'Max number of views for all objects', fontsize=20)
    
    means_sep = [x[2] for x in planner_num_views_separate]
    stds_sep = [x[3] for x in planner_num_views_separate]
    x = np.arange(len(planner_num_views_separate))
    fig_sep, ax_sep = plt.subplots()
    barlist_sep = ax_sep.bar(x, means_sep, width, color='r', yerr=stds_sep, ecolor='k')
    ax_sep.set_xlim(0,len(planner_names))
    ax_sep.set_xticks(x+width/2)
    xtickNames = ax_sep.set_xticklabels(planner_names)
    for b in range(len(barlist_sep)):
        barlist_sep[b].set_color(RGB[b])
        barlist_sep[b].set_edgecolor('black')
        if hatches[b]:
            barlist_sep[b].set_hatch('//')
    # Rotate the x labels
    for b in range(len(planner_names)):
        #xtickNames = axarr[view_count][object_count].set_xticklabels(class_names)
        plt.setp(ax_sep.xaxis.get_majorticklabels(), rotation=90, fontsize=20)
    # Adjust the plot size to fit the labels
    plt.tight_layout()
    # Add the values to the bars
    for ii, m in enumerate(means_sep):
        height = m
        ax_sep.text(x[ii]+width/2, height+0.2, '%0.1f'% m, fontsize=20, ha='center', va='bottom', backgroundcolor='w')
    plt.ylabel(r'Number of views', fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=20)
    plt.title(r'Number of views for objects separately', fontsize=20)
    
    save_min_views_file(means, stds)
        
def visualize_instance_entropy(directory):
    # Read the directory for all 'class_entropy' files
    _class_entropy_file = 'class_entropy_'
    class_entropies = []
    p_ix = 0
    while True:
        filename = directory + _class_entropy_file + str(p_ix) + '.txt'
        if os.path.isfile(filename):
            # Read the number in the file
            f = open(filename, 'r')
            val = f.read()
            class_entropies.append(val)
        else:
            break
        p_ix += 1
    x = range(0,len(class_entropies))
    f1, ax1 = plt.subplots(1,1)
    ax1.plot(x, class_entropies, linewidth=2)
    #ax1.set_title('Class Entropy')
    plt.xticks(np.arange(min(x), max(x), 1.0))
    plt.xlabel(r'View number', fontsize=20)
    plt.ylabel(r'Entropy', fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=20)
    
    # Read the directory for all 'self_probability' files
    _self_prob_file = 'self_prob_'
    self_probs = []
    p_ix = 0
    while True:
        filename = directory + _self_prob_file + str(p_ix) + '.txt'
        if os.path.isfile(filename):
            # Read the number in the file
            f = open(filename, 'r')
            val = f.read()
            self_probs.append(val)
        else:
            break
        p_ix += 1
    x = range(0,len(self_probs))
    f2, ax2 = plt.subplots(1,1)
    ax2.plot(x, self_probs, linewidth=2)
    #ax2.set_title('Classification Confidence')
    plt.xticks(np.arange(min(x), max(x), 1.0))
    plt.xlabel(r'View number', fontsize=20)
    plt.ylabel(r'Probability of Banana', fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=20)
            