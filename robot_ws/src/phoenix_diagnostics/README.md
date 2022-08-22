# Phoenix Diagnostics #

This package is responsible to report the current status of the ROS-Packages/algorithms and sensor drivers in phoenix (high-level and embedded) software.
This package will report the following info on the /diagnostics topic for each topic that we are interested in:

[Error_code conf page](https://ais-ugv2.atlassian.net/l/c/nq8B7gva)


Package capabilities in real-time for topic X: 

```
name: "phoenix_diagnostics: embedded/battery/current_draw topic status"
message: "Desired frequency met"
hardware_id: "TODO_ID"
values:
-
key: "error_code"
value: "000000"
-
key: "Events in window"
value: "54"
-
key: "Events since startup"
value: "4496"
-
key: "Duration of window (s)"
value: "10.805902"
-
key: "Actual frequency (Hz)"
value: "4.997269"
-
key: "Minimum acceptable frequency (Hz)"
value: "3.600000"
-
key: "Maximum acceptable frequency (Hz)"
value: "11.000000"

```

### How to run the package? ###
Once the software (Perceptioon, navigation, em bedded, and the app) is up and running, open a pane in the tmux terminal (phoenix-binder docker container) and:


```
source /opt/ros/noetic/setup.bash

source ~/robot_ws/instal/setup.bash

roslaunch phoenix_diagnostics phonix_diagnostics.launch


```


In order to see the real-time report:

```
source /opt/ros/noetic/setup.bash

source ~/robot_ws/instal/setup.bash

rostopic echo /diagnostics

```


Note: There will be related log INFO in the terminal as well.

### How to add another topic to the diagnostics package:

Example: let's say you want to add "test_topic" to the package.

1- Add the topic name and its diagnostics parameters to the params/diagnostics_params.yaml.

```

  test_topic:
    topic_name: "test_topic"
    error_code: "999999"
    error_msg: "TBD"
    freq_bounds: {'min':4, 'max':10}

```

2- Inside the phoenix_diagnostics/scripts/phoenix_diagnostics_node.py, every topic has its own diagnostics block in the initialization function. Let's add the diagnostics block for the "test_topic" in the node.

```
# Test Topic--------------------------------------
        self.test_updater = diagnostic_updater.Updater()
        self.test_updater.setHardwareID("TODO_ID" )
        self.test_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/test_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/test_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/test_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/test_topic/freq_bounds")}

        self.test_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.test_topic_info['topic_name'], self.test_updater,
            diagnostic_updater.FrequencyStatusParam(self.test_topic_info['freq_bounds'], 0.1, 10), self.test_topic_info['error_code'])
        self.test_updater.force_update()
        self.test_pub_freq.tick()
        self.test_updater.update()
        rospy.Subscriber(self.test_topic_info['topic_name'], Image, self.test_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.test_topic_info['topic_name'])

```

3- Add the "test-topic" callback function:

```
# Test Topic Callback Function
    def test_callback(self, data):
        self.test_pub_freq.tick()
        self.test_updater.update()

```


4- Add the diagnostics updater to the timer callback:

```
self.test_updater.update()

```

### Low frequency topics ###
if you have a low frequency topic, calculate the window size and minimum frequency bound and insert it in the param file (diagnostics_params.yaml). 

* window_size = Number of seconds that takes for your desired topic to get updated --> T

* min_frequency = 1/T


### Who do I talk to? ###

* Behnam Moradi(b.moradi@ai-systems.ca)