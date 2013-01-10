vfh_recognition_node provides a ROS node for object recognition and pose estimation. This information is intended to be used to look up an appropriate grasp from a database such as the household objects database. The node does these things:

-Subscribes to an openni device device.
-Provides a ROS service that takes a tabletop_object_detector::TabletopObjectRecognition srv
-When the services is called, does the following on most recent cloud:
--Finds and removes tabletop by RANSAC fitting
--Euclidean segmentation of remaining cloud
--For each segment:
---Extracts VFH feature
---Finds nearest neighbor in training set
---Loads .pcd for nearest neighbor
---Does RANSAC fit of nearest neighbor to cloud segment.
--Populates srv response which contains (among other things) a model_id and pose.

Notes:

To use this node, it must be trained. To train, put all training images in the "data" folder and run extract_vfh to extract vfh signatures and write them to file on all objects in "data". This is the only training that must be done offline. The remainder of the training is fast and happens when the node is instantiated.

The "data" file currently contains only a few samples of a random wood block. This is all that will be recognized until a larger training set is developed.

The model_id's are currently populated with garbage since no database currently exists in which these model_id's would have any meaning. I suggest that model_id's and poses be encoded into the filenames of the training models where they can be readily extracted and inserted into the srv_response when the object database has been created. The final pose inserted into the srv_response should be the transformation determined in this code premultiplied by the transformation associated with the training model. (i.e. the transofmation computed here is the transformation to the training model, NOT to camera coordinates.)

While there is a srv_request in the tabletop_object_detector::TabletopObjectRecognition srv, it is not used by this node. Any information in the request is completely ignored. 

Experience shows that VFH is not particularly good at pose estimation. Another version of this node will appear in the near future that uses a different feature better suited to instance-level recognition and pose estimation. In the future version, there will be no final RANSAC pose fit since the feature is viewpoint variant enough to obtain sufficient pose accuracy.