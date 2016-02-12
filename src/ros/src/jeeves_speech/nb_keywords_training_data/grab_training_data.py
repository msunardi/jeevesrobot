#!/usr/bin/env python

import os

output_document = ""

directory = './categories/'
for category in os.listdir(directory):
	for doc in os.listdir(directory + category):
		output_document += open(directory + category + '/' + doc).read() + "\n"

directory = './keywords/'
for group in os.listdir(directory):
	for doc in os.listdir(directory + '/' + group):
		output_document += open(directory + group + '/' + doc).read() + "\n"

open('for_lmtool.txt', 'w').write(output_document)
