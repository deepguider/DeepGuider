# -*- coding: utf-8 -*-
import pickle5 as pickle

with open('data_ocr/poinames_COEX.pickle', 'rb') as f:
	poinames = pickle.load(f)


from hanspell import spell_checker #py-hanspell
from soynlp.hangle import levenshtein
from soynlp.hangle import jamo_levenshtein
#from soynlp.hangle import compose
#from soynlp.hangle import decompose

def levenshtein(s1, s2, cost=None, debug=False):
    if len(s1) < len(s2):
        return levenshtein(s2, s1, debug=debug)

    if len(s2) == 0:
        return len(s1)

    if cost is None:
        cost = {}

    # changed
    def substitution_cost(c1, c2):
        if c1 == c2:
            return 0
        return cost.get((c1, c2), 1)

    previous_row = range(len(s2) + 1)
    for i, c1 in enumerate(s1):
        current_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = previous_row[j + 1] + 1
            deletions = current_row[j] + 1
            # Changed
            substitutions = previous_row[j] + substitution_cost(c1, c2)
            current_row.append(min(insertions, deletions, substitutions))

        if debug:
            print(current_row[1:])

        previous_row = current_row

    return previous_row[-1]

def get_neighbors(train, test_row, num_neighbors): #3. KNN 으로 가장 유사한 글자 찾기
	cost = {('액', '엑'):0.1} #코액스 -> 코엑스
	distances = list()
	for train_row in train:
		dist = levenshtein(test_row, train_row[0], cost) #2. string 거리 측정
		#dist = levenshtein(test_row, train_row) #2. string 거리 측정
		#dist = jamo_levenshtein(test_row, train_row) #2. string 거리 측정
		dist_conf = 1.0 - (dist / max(len(test_row), len(train_row[0])))
		#if dist_conf > 0.5:
		#	print(dist_conf, dist, test_row, train_row[0])
		distances.append((train_row[0], dist_conf))
		#distances.append((train_row, dist))
	distances.sort(key=lambda tup: tup[1], reverse=True)
	neighbors = list()
	for i in range(num_neighbors):
		#neighbors.append(distances[i][0])		
		neighbors.append((distances[i][0], distances[i][1]))
	return neighbors

def pred_filtering(pred):
	if len(pred) < 2:
		return pred, 0

	#print(pred)
	fix_spell = spell_checker.check(pred) #1. 한글 오탈자 보정
	fixed_string = fix_spell.checked
	#print(fixed_string)
	neighbors = get_neighbors(poinames, fixed_string, 1)
	result = neighbors[0][0]
	dist_conf = neighbors[0][1]
	#print(result, dist_conf)

	return result, dist_conf
