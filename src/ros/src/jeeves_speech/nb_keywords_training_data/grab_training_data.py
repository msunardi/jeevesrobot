#!/usr/bin/env python

import sys, os

# Set current working directory
cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)));
cwd = os.getcwd() + '/';

output_document = "";

# Process categories
directory = 'categories/';
for category in os.listdir(directory):
   for doc in os.listdir(directory + category):
      output_document += open(directory + category + '/' + doc).read() + "\n";

# Process keywords
directory = 'keywords/';
for group in os.listdir(directory):
   for doc in os.listdir(directory + '/' + group):
      output_document += open(directory + group + '/' + doc).read() + "\n";

# Open the cmu07a.dic file that contains a lot of words
#f = open('cmu07a.dic', 'r')
#words = f.read().split('\n')
#f.close()

# Loop through all lines and extract the actual word to be added to the new dictionary
#for line in words:
#	if(len(line) > 0):
#		tmp_wrd = line.replace('\t', '   ');
#		tmp_wrd = tmp_wrd.split(" ");
#		output_document += tmp_wrd[0] + '\n';
		
		
open('for_lmtool.txt', 'w').write(output_document)
