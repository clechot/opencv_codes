import sys
import os
import math
import random
import xml.etree.cElementTree as ET

nber_virtual_cam = sys.argv[2]
print("Number of virtual cameras: %d" %int(nber_virtual_cam))
print("File name: %s" %str(sys.argv[1]))

with open(str(sys.argv[1]), 'r+') as rwlaunchfile:
    rwlaunchfile.seek(0, os.SEEK_END)
    pos = rwlaunchfile.tell() - 1
    while pos > 0 and rwlaunchfile.read(1) != "\n":
        pos -= 1
        rwlaunchfile.seek(pos, os.SEEK_SET)
    if pos > 0:
        rwlaunchfile.seek(pos, os.SEEK_SET)
        rwlaunchfile.truncate()

for index_camera in range(2, int(nber_virtual_cam)+1):
    value_x = random.uniform(-5, 5)
    value_y = random.uniform(-5, 5)
    value_z = random.uniform(0, 2)
    value_angle = random.uniform(-math.pi, math.pi)
    with open(str(sys.argv[1]), 'a') as launchfile:
        launchfile.write('\n')
        param = ET.Element("param",name="camera_description" + str(index_camera),command="$(find xacro)/xacro.py '$(arg urdf_path)/$(arg camera_urdf_name)' camera_name:=synthetic_camera" + str(index_camera))
        node = ET.Element("node",name="spawn_camera" + str(index_camera),pkg="gazebo_ros",type="spawn_model",
            args="-param camera_description" + str(index_camera) + " -urdf -model synthetic_camera_" + str(index_camera) + " -x " + str(value_x) + " -y " + str(value_y) + " -z " + str(value_z) + " -Y " + str(value_angle) + " -P 0.15")
        tree_p = ET.ElementTree(param)
        launchfile.write('  ')
        tree_p.write(launchfile)
        launchfile.write('\n')
        tree_n = ET.ElementTree(node)
        launchfile.write('  ')
        tree_n.write(launchfile)
        launchfile.write('\n')
        launchfile.close()

with open(str(sys.argv[1]), 'a') as launchfile:
    launchfile.write('\n')
    launchfile.write('</launch>\n')

