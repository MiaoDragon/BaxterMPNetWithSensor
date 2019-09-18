"""
a small tool that parses the urdf file and return the link name in baxter_links.yaml
"""

file = open('../../baxter_common/baxter_description/urdf/baxter.urdf', 'r')
link_names = []
for line in file:
    # find pattern: <link name="">
    if line.find('<link') != -1:
        start = line.find('name=')
        i = start + 6
        link_name = ''
        while line[i] != '\"':
            link_name+=line[i]
            i += 1
        link_names.append(link_name)
# save to a yamal file
file.close()
# add electric gripper
link_names += ['left_gripper_l_finger', 'left_gripper_l_finger_tip', 'left_gripper_r_finger', 'left_gripper_r_finger_tip',
               'right_gripper_l_finger', 'right_gripper_l_finger_tip', 'right_gripper_r_finger', 'right_gripper_r_finger_tip']


file = open('baxter_links.yaml', 'w')
file.write('self_see_links: [')
for name in link_names:
    file.write('{"name":"%s","padding":0.18,"scale":0.1},\n' % (name))
file.write(']')
