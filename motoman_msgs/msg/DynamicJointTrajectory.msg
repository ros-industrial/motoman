#length: true message/data length
#header: standard msg_type, comms_type, reply_code fields
#sequence:
#num_groups: # of motion groups included in this message
#group[]: DynamicJointPoint from DynamicJointPoint.msg

Header header
string[] joint_names
DynamicJointPoint[] points
