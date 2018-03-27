from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import cv2
import os

graph_filename = "light_classification/frozen_inference_graph.pb"

# for testing traffic light identification
test_images_folder = "light_classification/test/"
img_save_count = 0
fileHandler = 0

def load_graph(frozen_graph_filename):
    """Loads the graph from the file

    Args:
        frozen_graph_filename: The filemane of the graph

    Returns:
        graph: the loaded graph
    """
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read()) 
    with tf.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name="")
    return graph

def crop_to_rect(image, bbox):
    """Crops the image to the specified bounding box

    Args:
        image (cv::Mat): the full image
        bbox: the boundaries to crop

    Returns:
        image: the cropped image
    """
    rows = image.shape[0]
    cols = image.shape[1]
    top = int(bbox[0] * rows)
    left = int(bbox[1] * cols)
    bottom = int(bbox[2] * rows)
    right = int(bbox[3] * cols)
    return image[top:bottom, left:right]

def record_tl_bounding_boxes(image, bboxes, num_detections, scores, classes, saveImg=False):
    global img_save_count

    # Record bounding boxes
    rows = image.shape[0]
    cols = image.shape[1]
    for i in range(num_detections):
        bbox = [float(v) for v in np.squeeze(bboxes)[i]]
        top = int(bbox[0] * rows)
        left = int(bbox[1] * cols)
        bottom = int(bbox[2] * rows)
        right = int(bbox[3] * cols)
        score = np.squeeze(scores)[i]
        classId = int(np.squeeze(classes)[i])
        saveRow(img_save_count, score, classId, top, left, bottom, right)

    # Save image
    if saveImg:
        cv2.imwrite(test_images_folder + str(img_save_count) + ".jpg", image)
        img_save_count += 1

def saveRow(num, score, classId, top, bottom, left, right):
    global fileHandler
    fileHandler.write(str(num) + ',' + str(score) + ',' + str(classId) + ',' + str(top) + ',' + str(bottom) + ',' + str(left) + ',' + str(right) + '\n')

def newFile(filename):
    global fileHandler
    fileHandler = open(test_images_folder + filename, "a+")
    fileHandler.write("image, score, classId, top, left, bottom, right\n")

def get_traffic_lights(image, graph, test=False):
    """Finds the traffic lights in the image, if any

    Args:
        image (cv::Mat): image containing the traffic light
        graph: the graph for classification
        [optional] test (boolean): when True, saves cropped images to a file for testing purposes

    Returns:
        array: array containing the cropped traffic light images
    """
    with tf.Session(graph=graph) as sess:
        tf_image_input = np.expand_dims(image, axis=0)

        detections, scores, boxes, classes = sess.run([
            sess.graph.get_tensor_by_name('num_detections:0'),
            sess.graph.get_tensor_by_name('detection_scores:0'),
            sess.graph.get_tensor_by_name('detection_boxes:0'),
            sess.graph.get_tensor_by_name('detection_classes:0')], 
            feed_dict={'image_tensor:0': tf_image_input})

        num_detections = int(np.squeeze(detections))

        # for training, record bounding boxes
        record_tl_bounding_boxes(image, boxes, num_detections, scores, classes, saveImg=True)

        images = []
        for i in range(num_detections):
            classId = int(np.squeeze(classes)[i])
            if (classId != 10):
                continue
            score = np.squeeze(scores)[i]
            bbox = [float(v) for v in np.squeeze(boxes)[i]]

            if score > 0.3:
                #print ("Class: {}, Score: {}".format(classId, score))
                images.append(image)

                # crop image and add to images array
                # cropped = crop_to_rect(image, bbox)
                # images.append(cropped)

                if test:
                    # for testing purposes save cropped images
                    global img_save_count
                    cv2.imwrite(test_images_folder + "crop_" + str(img_save_count) + ".png", cropped)
                    img_save_count += 1

        return images

class TLClassifier(object):
    def __init__(self):
        self.graph = load_graph(graph_filename)

        # for training dataset TODO remove
        newFile('datapts.csv')

        self.label_dict = {
            1: TrafficLight.RED,
            2: TrafficLight.YELLOW,
            3: TrafficLight.GREEN,
            4: TrafficLight.UNKNOWN
        }

        self.prediction_threshold = 0.5
        self.frozen_graph_path = os.getcwd() + '/light_classification/models/output_inference/frozen_inference_graph.pb'
        self.graph = load_graph(self.frozen_graph_path)
        self.sess = tf.Session(graph=self.graph)

        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # get the cropped traffic light images (TODO for testing)
        # lights = get_traffic_lights(image, self.graph, test=True)
        lights = get_traffic_lights(image, self.graph)

        # Implement light color prediction
        # Valid output: TrafficLight.UNKNOWN, RED, GREEN, YELLOW
        tf_image_input = np.expand_dims(image, axis=0)
        detections, scores, boxes, classes = self.sess.run(
            [self.num_detections, self.detection_scores, self.detection_boxes, self.detection_classes],
            feed_dict={self.image_tensor: tf_image_input})

        num_detections = int(np.squeeze(detections))
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score = self.prediction_threshold
        class_id = TrafficLight.UNKNOWN

        for i in range(num_detections):
            score = scores[i]
            if score > min_score:
                min_score = score
                class_id = self.label_dict[classes[i]]

        return class_id