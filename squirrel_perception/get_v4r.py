#! /usr(bin/env python

import re
import urllib2

url='http://lcas.lincoln.ac.uk/repos/release/pool/main/r/ros-indigo-v4r/'

def main():
    try: 
        response = urllib2.urlopen(url)
        content = response.read()
    except  urllib2.URLError, e:
        print('Unable to retrieve index file from server. Error: {}'.format(e))
        return
    matches = re.search(r'(>)(ros-indigo-v4r.*amd64.deb)', content)
    if not matches:
        print('Unable to find link to .deb file in html ')
        return
    file = matches.group(2)
    print(url+file)

    response = urllib2.urlopen(url+file)
    with open(file, 'wb') as f:
        f.write(response.read())
    print('done')


if __name__ == '__main__':
    print('Downloads the latest .deb file for v4r from STRANDS project repository')
    main()
