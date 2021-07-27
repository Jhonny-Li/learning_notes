import rosbag
import copy


with rosbag.Bag('tf_cheku.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('cheku.bag').read_messages():
        if topic == "/tf":
            tras_t265 =  copy.deepcopy(msg.transforms[0])
            tras_d435=  copy.deepcopy(msg.transforms[0])

            msg.transforms = [elem for elem in msg.transforms if elem.child_frame_id  == "t265_pose_frame"]

            tras_t265.header.frame_id = "t265_pose_frame"
            tras_t265.child_frame_id  = "d435i_link"
            tras_t265.transform.translation.x = 0.0
            tras_t265.transform.translation.y = 0.0
            tras_t265.transform.translation.z = 0.0
            tras_t265.transform.rotation.x = 0.0
            tras_t265.transform.rotation.y = -0.382
            tras_t265.transform.rotation.z = 0.0
            tras_t265.transform.rotation.w = 0.924
            msg.transforms.append(tras_t265)

            tras_d435.header.frame_id = "d435i_link"
            tras_d435.child_frame_id  = "d435i_color_optical_frame"
            tras_d435.transform.translation.x = 0.0
            tras_d435.transform.translation.y = 0.0
            tras_d435.transform.translation.z = 0.0
            tras_d435.transform.rotation.x = 0.506
            tras_d435.transform.rotation.y = -0.496
            tras_d435.transform.rotation.z = 0.502
            tras_d435.transform.rotation.w = -0.496
            msg.transforms.append(tras_d435)

            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)
