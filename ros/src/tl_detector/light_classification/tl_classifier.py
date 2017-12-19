from styx_msgs.msg import TrafficLight
import rospy
import rospkg
import numpy as np
import os
import sys
import tensorflow as tf
import time
import rospkg

class TLClassifier(object):
    def __init__(self):
        rp = rospkg.RosPack()
        model_dir = os.path.join(rp.get_path('tl_detector'), 'models')
        rospy.loginfo('Using model directory {}'.format(model_dir))

        model_name = rospy.get_param('~model_name', 'sim.pb')
        model_path = os.path.join(model_dir, model_name)

        if not os.path.exists(model_path):
            rospy.logerr('Model not found at {}'.format(model_path))

        self.current_light = TrafficLight.UNKNOWN

        self.image_np_deep = None
        self.detection_graph = tf.Graph()

        # Activate optimizations for TF
        if rospy.get_param('~tf_optimize', False):
            self.config = tf.ConfigProto(device_count={'GPU': 1, 'CPU': 1})
            self.config.gpu_options.allow_growth = True
            self.config.gpu_options.per_process_gpu_memory_fraction = 0.8
        else:
            self.config = tf.ConfigProto()

        jit_level = tf.OptimizerOptions.ON_1
        self.config.graph_options.optimizer_options.global_jit_level = jit_level

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=config)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.lightIndex = {0: TrafficLight.UNKNOWN, 1: TrafficLight.GREEN, 2: TrafficLight.RED,  3: TrafficLight.YELLOW}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        self.current_light = TrafficLight.UNKNOWN

        image_expanded = np.expand_dims(image, axis=0)
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                 self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        score = -1
        min_score_thresh = .60
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                if classes[i] > 3:
                    self.current_light = TrafficLight.UNKNOWN
                    score = 1.0
                else:
                    self.current_light = self.lightIndex[classes[i]]
                    score = scores[i]

                self.image_np_deep = image

        return self.current_light, score