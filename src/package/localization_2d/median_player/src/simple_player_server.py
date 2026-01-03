#!/usr/bin/env python

# Author: lei.zeng@tu-dortmund.de

import rospy
import actionlib
import os
import random
from playsound import playsound
from median_player.msg import SimplePlayerAction, SimplePlayerActionFeedback


class SimplePlayerServer:
    def __init__(self):
        self._file_path = rospy.get_param(
            "/simple_player_server/file_path", '/home/zeng/Downloads/music')  # '/home/zeng/catkin_ws/src/median_player/sound'
        self._action_name = 'simple_player_server'
        self._feedback = SimplePlayerActionFeedback()
        self._action_server = actionlib.SimpleActionServer(
            self._action_name, SimplePlayerAction, execute_cb=self._execute_cb, auto_start=False)
        self._action_server.start()

    def _preempt_request(self):
        if self._action_server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._action_server.set_preempted()
            return False
        else:
            return True

    def _feedback_pub(self, idex):
        self._feedback.feedback.playing_id = idex
        self._action_server.publish_feedback(self._feedback.feedback)

    def play_sound_once(self, file_path, sound_id):
        sound_path = file_path+'/' + str(sound_id)+'.ogg'
        try:
            playsound(sound_path)
        except:
            rospy.logwarn(
                '[Simple Player] Cannot play the sound: %s. The file may not exist.' % sound_path)

    def _execute_cb(self, goal):
        success = True
        # Single Loop : MODE = 0, ID = id
        if goal.mode == 0 and goal.sound_id > 0:
            for i in range(0, 2625):
                success = self._preempt_request()
                if not success:
                    break

                self._feedback_pub(goal.sound_id)
                self.play_sound_once(self._file_path, goal.sound_id)

        # Sequential loops: MODE = 0, ID = 0
        elif goal.mode == 0 and (goal.sound_id == 0 or goal.sound_id == -1):
            for i in range(0, 2625):
                # cd_cmd = 'find '+ self._file_path +' -type f |wc -l'
                # os.system(cd_cmd)
                # for single in  os.listdir(self._file_path):

                sound_num = len([name for name in os.listdir(
                    self._file_path) if os.path.isfile(os.path.join(self._file_path, name))])
                for single_id in range(1, sound_num+1):
                    success = self._preempt_request()
                    if not success:
                        break

                    if goal.sound_id == -1:
                        sid = random.randint(1, sound_num)
                    else:
                        sid = single_id

                    self._feedback_pub(sid)
                    self.play_sound_once(self._file_path, sid)

                success = self._preempt_request()
                if not success:
                    break

        # Default Single: MODE = 1
        elif goal.mode == 1:
            self._preempt_request()
            if goal.sound_id == 0:
                goal.sound_id += 1
            self._feedback_pub(goal.sound_id)
            self.play_sound_once(self._file_path, goal.sound_id)
        else:
            rospy.logwarn('[Simple Player] Invalid Request')

        if success:
            play_finished = 1
            rospy.loginfo('%s: Finished' % self._action_name)
            self._action_server.set_succeeded(play_finished)


def main():
    rospy.init_node('simple_player_server')
    SimplePlayerServer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down ... ')


if __name__ == '__main__':
    main()
