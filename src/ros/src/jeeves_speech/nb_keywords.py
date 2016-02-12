# nb_keywords.py - naive bayes classifier in form P(category|words) = P(words|category) * P(category)
# with Laplace smoothing - P(word|category) = wordfrequency + a / #classvocabulary + (a * #allvocab),
# where a is a smoothing factor (=1 for Laplace smoothing)
# additional assumptions: only count each word once when calculating word totals in vocab and categories
# additional assumptions: assume that if there is only one document for each category, then each category has the same base probability

import re
import math
import os
import string
from operator import itemgetter
from itertools import islice

def sanitize(text):
	table = string.maketrans("","")
	text = text.translate(table, string.punctuation)
	text = text.lower()
	return text
	
# function to create a dictionary of word counts from a text
def make_word_count(text):
	word_count = {}
	for word in re.split("\W+", sanitize(text)):
		# do not count or add to dictionary words that are 3 or less characters in length
#		if len(word) <= 3:
#			continue
		word_count[word] = word_count.get(word, 0.0) + 1.0
	return word_count

vocab = {}
p_word_given_category = {}
p_category = {}
keywords = {}

# make categories from training data
def make_probabilities(directory = './nb_keywords_training_data/categories/'):
	category_word_counts = {}
	for category in os.listdir(directory):
		category_word_counts[category] = {}
		p_word_given_category[category] = {}
		p_category[category] = 0.0
		# make word count of documents in ./samples/*category_directory*
		for doc in os.listdir(directory + category):
			counts = make_word_count(open(directory + category + '/' + doc).read())
			# enter words and counts into the vocab and category_word_counts
			for word, count in counts.items():
				if word not in vocab:
					vocab[word] = 0.0
				if word not in category_word_counts[category]:
					category_word_counts[category][word] = 0.0
					# count words in p_category
					p_category[category] += 1
				vocab[word] += 1  # "vocab[word] += count" if you want to count words more than once 
				category_word_counts[category][word] += 1  # "category_word_counts[category][word] += count" if you want to count words more than once
	#calculate P(word|category) for each word in vocab into each category
	for word, count in vocab.iteritems():
		for category in category_word_counts:
			p_word_given_category[category][word] = (category_word_counts[category].get(word, 0.0) + 1) / (sum(category_word_counts[category].values()) + sum(vocab.values()))
	# finish calculating p_category
	numwords = sum(p_category.values())
	for category in p_category:
		p_category[category] = p_category[category] / numwords

# make keywords from training data
def make_keywords(directory = './nb_keywords_training_data/keywords/'):
	for group in os.listdir(directory):
		for doc in os.listdir(directory + '/' + group):
			keywords[group] = [sanitize(line).strip() for line in open(directory + group + '/' + doc)]
				
def get_keywords(text):
	s = sanitize(text)
	ret_keywords = {}
	for group in keywords:
		ret_keywords[group] = []
		for item in keywords[group]:
			if s.find(item) > -1:
				s = s.replace(item, group)
				ret_keywords[group].append(item)
		if len(ret_keywords[group]) == 0:
			del ret_keywords[group]
	return (s, ret_keywords)
 

# return 'maximum likelihood' category given text
def get_category(s, bestofn=1):
	# initialize dictionaries
	p_category_given_words = {}
	for category in p_word_given_category:
		p_category_given_words[category] = 0.0
	
	# for every word in our test sentence, make P(word|category) for each category,
	# compound them and then add P(category) to give P(category|words in test sentence)
	for word, count in make_word_count(s).items():
		# skip word if not in vocab
		if not word in vocab:
			continue
		for category in p_word_given_category:
			# add (log) probabilities to our running count, giving probability given *category*
			if p_word_given_category[category].get(word, 0.0) > 0.0:
				# note: x * log(z) == log(z^x), i.e. a test word repeated in the sample counts as several features
				p_category_given_words[category] += count * math.log(p_word_given_category[category][word])
	# add p_category and then convert probabilities from log
	for category in p_category_given_words:
		p_category_given_words[category] = math.exp(p_category_given_words[category]+math.log(p_category[category]))
	#return results
	return list(islice(sorted(p_category_given_words.items(), key=itemgetter(1), reverse=True), bestofn))
