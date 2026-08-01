Grasp attach plugin (suggested)

Use an attach/detach plugin to avoid relying on contact friction for grasp stability.
This does not modify the robot or gripper URDFs; it attaches the picked object to the gripper at runtime.

Example model plugin SDF snippet (add to the picked object's SDF/model stanza or spawn-time SDF):

<plugin name="link_attacher" filename="libgazebo_ros_link_attacher.so">
  <!-- No required parameters for the common link_attacher plugin; it exposes ROS services
       /link_attacher_node/attach and /link_attacher_node/detach that you can call to attach by name. -->
</plugin>

Alternative: If you have `gazebo_grasp_fix` available, a minimal plugin snippet could look like:

<plugin name="grasp_fix" filename="libgazebo_grasp_fix.so">
  <attach_service>/demo/attach</attach_service>
  <detach_service>/demo/detach</detach_service>
</plugin>

Where to put it
- If you control the spawned object's SDF (the demo cube in `teleop.py`), insert the plugin inside the `<model>` before `</model>`.
- Or add the plugin to the world file so the service is available globally.

How to use
- After the gripper closes in the demo, call the attach service (provided by the plugin) to rigidly attach the object's collision link to the gripper link.
- On release, call the detach service. This stabilizes the grasp without changing physical contact parameters.

Safety: This does not edit arm or gripper URDFs, contact/friction, or controller configs. It only creates a runtime rigid attachment between existing links.

Expected effect on RTF: low-to-medium (stabilises simulation spikes caused by contact chatter). Depends on implementation and frequency of attach/detach calls.
