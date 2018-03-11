from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import cv2

graph_filename = "ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb"

# for testing image cropping
test_images_folder = "cropped/"
img_save_count = 0

def load_graph(frozen_graph_filename):
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read()) 
    with tf.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name="")
    return graph

# needed?
def image_to_bgr(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

def crop_to_rect(image, bbox):
    rows = image.shape[0]
    cols = image.shape[1]
    left = bbox[1] * cols
    top = bbox[0] * rows
    right = bbox[3] * cols
    bottom = bbox[2] * rows
    return image[left:right, top:bottom]

def get_traffic_lights(image):
    """Finds the traffic lights in the image, if any

    Args:
        image (cv::Mat): image containing the traffic light

    Returns:
        lights (array): array containing the cropped traffic light images
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
        images = []

        for i in range(num_detections):
            classId = int(np.squeeze(classes)[i])
            if (classId != 10):
                continue
            score = np.squeeze(scores)[i]
            bbox = [float(v) for v in np.squeeze(boxes)[i]]

            if score > 0.3:
                #print ("Class: {}, Score: {}".format(classId, score))
                cropped = crop_to_rect(image, bbox)
                images.append(cropped)

                # for testing purposes save cropped images
                plt.imshow(cropped)
                plt.savefig(test_images_folder + "crop_" + img_save_count)
                img_save_count += 1

        return images

class TLClassifier(object):
    def __init__(self):
        self.graph = load_graph(graph_filename)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # get the cropped traffic light images
        lights = get_traffic_lights(image)


        #TODO implement light color prediction
        # Valid: TrafficLight.UNKNOWN, RED, GREEN, YELLOW
        return TrafficLight.UNKNOWN
