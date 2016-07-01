#!/usr/bin/env python

import sys, os

# Control
ADD_ENGL = False;

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
      

print ADD_ENGL

if ADD_ENGL:
   # Open the cmu07a.dic file that contains a lot of words
   f = open('cmu07a.dic', 'r')
   words = f.read().split('\n')
   f.close()

   # Loop through all lines and extract the actual word to be added to the new dictionary
   for line in words:
      if(len(line) > 0):
         if(line.find('(') == -1 and line.find(')') == -1 and line.find('{') == -1 and line.find('}') == -1 and line.find('\'') == -1 and line.find('\"') == -1 and line.find('!') == -1 and line.find('.') == -1):
            tmp_wrd = line.replace('\t', '   ');
            tmp_wrd = tmp_wrd.split(" ");
            output_document += tmp_wrd[0] + '\n';
         else:
            print "Skipped:  %s" % line

# Strip white space
output_document = output_document.strip();
         
# Sort all of the entries
output_document = output_document.replace('\r\n', '\n');
tmp_output_document = output_document.split('\n');
tmp_output_document.sort();
new_output = [];

# Remove lines with nothing but a newline character
for entry in tmp_output_document:
   if(len(entry) > 0):
      new_output.append(entry)
   else:
      print "Skipped newline...";
   
      
output_document = '\n'.join(new_output)
      
open('jeeves.txt', 'w').write(output_document)
