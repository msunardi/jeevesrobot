#!/bin/bash

# ------------ Create jeeves.txt for Language Tools ---------------
cd ~/projects/jeeves_speech/src/ros/src/jeeves_speech
python ~/projects/jeeves_speech/src/ros/src/jeeves_speech/dictionary/grab_training_data.py
cp ~/projects/jeeves_speech/src/ros/src/jeeves_speech/dictionary/jeeves.txt ~/lmtool/trunk/logios/Tools/MakeDict/jeeves/jeeves.txt

# --------- Generate Dictionary ".dic" file from lmtool -----------
cd ~/lmtool/trunk/logios/Tools/MakeDict
./jeeves.sh
cp jeeves/jeeves.dict ~/projects/jeeves_speech/src/ros/src/jeeves_speech/jeeves.dic

# ------------------ Generate Language Model ----------------------
cd ~/projects/jeeves_speech/src/ros/src/jeeves_speech
#ngram-count -kndiscount -interpolate -text dictionary/jeeves.txt -lm jeeves_unsorted.lm
ngram-count -interpolate -text dictionary/jeeves.txt -lm jeeves_unsorted.lm

# --------------- Reduce size of language model -------------------
ngram -lm jeeves_unsorted.lm -prune 1e-8 -write-lm jeeves_unsorted_pruned.lm

# --------------- Sort Language Model File ------------------------
sort-lm jeeves_unsorted_pruned.lm > jeeves.lm

# -------------- Test Perplexity of Language Model ----------------
ngram -lm jeeves.lm -ppl dictionary/jeeves.txt

echo Done!