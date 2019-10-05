from imutils import paths
import argparse
import time
import sys
import cv2
import os

parse = argparse.ArgumentParser()
parse.add_argument('--haystack', required=True,
	help='dataset of imgs to search')
parse.add_argument('--needles', required=True,
	help='set of imgs to be searched for')
args = vars(parse.parse_args())

stackPath = list(paths.list_images(args['haystack']))
needPath = list(paths.list_images(args['needles']))
print(stackPath)
matchedImgs = []

haystack = {}
start = time.time()

if not sys.platform[0] == 'w':
	stackPath = [path.replace('\\','') for path in stackPath]
	needPath = [path.replace('\\','') for path in needPath]

def dhash(image, hashSize=8):
	resized = cv2.resize(image,(hashSize+1,hashSize))
	diff = resized[:,1:] > resized[:,:-1]
	return sum([2**i for (i,v) in enumerate(diff.flatten()) if v])

for path in stackPath:
	img = cv2.imread(path)
	if img is None:
		continue
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	imgHash = dhash(img)
	matchList = haystack.get(imgHash, [])
	matchList.append(path)
	haystack[imgHash] = matchList
	
print("[INFO] processed %s images in %.2f seconds" % (len(haystack), 
	time.time()-start))

for path in needPath:
	img = cv2.imread(path)
	if img is None:
		continue
	img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	imgHash = dhash(img)
	if haystack[imgHash]:
		matchedImgs.append(path)

print("[INFO] check the following images:\n")
for path in matchedImgs:
	print("--> %s\n" % path)
	
	
	
	



