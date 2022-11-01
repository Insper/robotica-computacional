import numpy as np
import argparse
import cv2 as cv
import subprocess
import time
import os
from yolo_utils import infer_image, show_image
import imutils

FLAGS = []


"""

Adapted from:

https://github.com/iArunava/YOLOv3-Object-Detection-with-OpenCV/blob/master/yolo.py#L154

"""


def print_categories(boxes, confidences, classids, labels):
	""" 
	Nota do Miranda: 
		Função estratégica para modificar/ usar o Yolo
	"""
	for i in range(len(classids)):
		print(labels[classids[i]], confidences[i], boxes[i])



if __name__ == '__main__':
	parser = argparse.ArgumentParser()

	parser.add_argument('-m', '--model-path',
		type=str,
		default='./yolov3-coco/',
		help='The directory where the model weights and \
			  configuration files are.')

	parser.add_argument('-w', '--weights',
		type=str,
		default='./yolov3-coco/yolov3.weights',
		help='Path to the file which contains the weights \
			 	for YOLOv3.')

	parser.add_argument('-cfg', '--config',
		type=str,
		default='./yolov3-coco/yolov3.cfg',
		help='Path to the configuration file for the YOLOv3 model.')

	parser.add_argument('-i', '--image-path',
		type=str,
		help='The path to the image file')

	parser.add_argument('-v', '--video-path',
		type=str,
		help='The path to the video file')


	parser.add_argument('-vo', '--video-output-path',
		type=str,
        default='./output.avi',
		help='The path of the output video file')

	parser.add_argument('-l', '--labels',
		type=str,
		default='./yolov3-coco/coco-labels',
		help='Path to the file having the \
					labels in a new-line seperated way.')

	parser.add_argument('-c', '--confidence',
		type=float,
		default=0.5,
		help='The model will reject boundaries which has a \
				probabiity less than the confidence value. \
				default: 0.5')

	parser.add_argument('-th', '--threshold',
		type=float,
		default=0.3,
		help='The threshold to use when applying the \
				Non-Max Suppresion')

	parser.add_argument('--download-model',
		type=bool,
		default=False,
		help='Set to True, if the model weights and configurations \
				are not present on your local machine.')

	parser.add_argument('-t', '--show-time',
		type=bool,
		default=False,
		help='Show the time taken to infer each image.')

	FLAGS, unparsed = parser.parse_known_args()

	# Download the YOLOv3 models if needed
	if FLAGS.download_model:
		subprocess.call(['./yolov3-coco/get_model.sh'])

	# Get the labels
	labels = open(FLAGS.labels).read().strip().split('\n')

	# Known categories
	print("Known categories: ", " ".join(labels))

	# Intializing colors to represent each label uniquely
	colors = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')

	# Load the weights and configutation to form the pretrained YOLOv3 model
	net = cv.dnn.readNetFromDarknet(FLAGS.config, FLAGS.weights)

	# Get the output layer names of the model
	layer_names = net.getLayerNames()
	layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        


	vid = cv.VideoCapture(0)

	count = 0

	while True:
		_, frame = vid.read()

		frame = imutils.resize(frame, width=600)


		height, width = frame.shape[:2]

		if count == 0:
			frame, boxes, confidences, classids, idxs = infer_image(net, layer_names, \
								height, width, frame, colors, labels, FLAGS)
			count += 1
		else:
			frame, boxes, confidences, classids, idxs = infer_image(net, layer_names, \
								height, width, frame, colors, labels, FLAGS, boxes, confidences, classids, idxs, infer=False)
			count = (count + 1) % 6

		print_categories(boxes, confidences, classids, labels)

		cv.imshow('webcam', frame)

		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	vid.release()
	cv.destroyAllWindows()
