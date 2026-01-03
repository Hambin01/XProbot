#!/usr/bin/env python

# Author: lei.zeng@tu-dortmund.de

import rospy
import time
import actionlib
import os
import random
import vlc
from median_player.msg import MedianPlayAction, MedianPlayActionGoal, MedianPlayActionFeedback, MedianPlayResult
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID


class MedianPlayerServer:
    def __init__(self):
        self._control_free = True
        self._file_type = rospy.get_param(
            "median_player_server/file_type", '.ogg')
        self._volume_step = rospy.get_param(
            "median_player_server/volume_step", 30)
        self._median_path = rospy.get_param(
            "median_player_server/median_path", '/home/zeng/Music')  # '/home/zeng/catkin_ws/src/median_player/sound'
        if_stop_sofort = rospy.get_param(
            "median_player_server/if_stop_sofort", False)
        self._action_name = 'median_player_server'
        self._feedback = MedianPlayActionFeedback()
        self._action_server = actionlib.SimpleActionServer(
            self._action_name, MedianPlayAction, execute_cb=self._execute_cb, auto_start=False)
        self._action_server.start()
        self._sub_goal = rospy.Subscriber(
            'median_control', Header, self._detectPlayControlCallback, queue_size=1)
        if if_stop_sofort:
            self._sub_cancel = rospy.Subscriber(
                'median_player_server/cancel', GoalID, self._cancelCallback, queue_size=1)

    def _cancelCallback(self, msg):
        try:
            self._player.stop()
            rospy.logdebug('[MP] Stopping playing')
        except:
            pass

    def _detectPlayControlCallback(self, ctrlMsg):
        if (ctrlMsg.frame_id).lower() == 'pause':
            try:
                if self._player.is_playing() == 1:
                    self._player.pause()
                    self._control_free = False
                    rospy.logdebug('[MP] Pausing player')
                else:
                    rospy.logdebug('[MP] Already paused')
            except:
                pass
        elif (ctrlMsg.frame_id).lower() == 'set_pause':
            try:
                self._player.set_pause(0)
                self._control_free = True
                rospy.logdebug('[MP] Resetting player')
            except:
                pass

        elif (ctrlMsg.frame_id).lower() == 'play':
            try:
                self._player.play()
                time.sleep(0.5)
                rospy.logdebug('[MP] Playing player')
                self._control_free = True
            except:
                pass
        elif (ctrlMsg.frame_id).lower() == 'skip':
            try:
                self._player.stop()
                self._control_free = True
            except:
                pass
        elif (ctrlMsg.frame_id).lower() == 'up':
            try:
                volume = self._player.audio_get_volume()
                self._player.audio_set_volume(volume + self._volume_step)
            except:
                pass
        elif (ctrlMsg.frame_id).lower() == 'down':
            try:
                volume = self._player.audio_get_volume()
                self._player.audio_set_volume(volume - self._volume_step)
            except:
                pass
        elif (ctrlMsg.frame_id).lower() == 'volume':
            try:
                self._player.audio_set_volume(ctrlMsg.seq)
            except:
                pass

    def _preempt_request(self):
        if self._action_server.is_preempt_requested():
            # TO BE OR NOT TO BE
            # try:
            #     self._player.stop()
            # except:
            #     pass
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._action_server.set_preempted()
            return False
        else:
            return True

    def _feedback_pub(self, idex):
        self._feedback.feedback.playing_id = idex
        self._action_server.publish_feedback(self._feedback.feedback)

    def _play_once_completly(self, file_path, sound_id):
        try:
            self._player.stop()
        except:
            pass

        self._feedback_pub(sound_id)

        start_flag = False
        complete_flag = False
        sound_path = file_path+'/' + str(sound_id)+self._file_type

        # self._player = vlc.MediaPlayer(sound_path)
        self._player = vlc.MediaPlayer()
        self._player.set_mrl(sound_path)

        while not complete_flag:
            if not start_flag and self._player.is_playing() != 1:
                self._player.play()
                rospy.logdebug('[MP] Starting playing: %s' % sound_path)
                start_flag = True
                time.sleep(0.5)
            if start_flag and self._player.is_playing() != 1 and self._control_free:
                complete_flag = True
                self._player.stop()
                rospy.logdebug('[MP] Finish playing: %s' % sound_path)
                time.sleep(0.5)
            time.sleep(0.1)
        return complete_flag

    def _execute_cb(self, goal):
        success = True
        if not goal.path == '':
            self._median_path = goal.path
        median_num = len([name for name in os.listdir(
            self._median_path) if os.path.isfile(os.path.join(self._median_path, name))])
        # self._player = vlc.MediaPlayer(self._median_path)

        # SINGLE LOOP : MODE = 'loop', ID = id
        if (goal.mode).lower() == 'loop' and 0 < goal.sound_id:
            sound_path = self._median_path+'/' + \
                str(goal.sound_id)+self._file_type
            i = 0
            while i < 2625:
                success = self._preempt_request()
                if not success:
                    break
                if self._play_once_completly(self._median_path, goal.sound_id):
                    i = i+1
                time.sleep(0.1)

        # SEQUENTIAL LOOP: MODE = 'loop', ID = 0
        elif ((goal.mode).lower() == 'loop' or (goal.mode).lower() == 'rand') and goal.sound_id == 0:
            for i in range(0, 2625):
                # cd_cmd = 'find '+ self._median_path +' -type f |wc -l'
                # os.system(cd_cmd)
                # for single in  os.listdir(self._median_path):
                single_id = 1
                while single_id <= median_num:
                    success = self._preempt_request()
                    if not success:
                        break
                    temp_id = single_id
                    # Random LOOP: MODE = 'rand', ID = 0
                    if (goal.mode).lower() == 'rand':
                        temp_id = random.randint(1, median_num)
                    if self._play_once_completly(self._median_path, temp_id):
                        single_id = single_id + 1
                    time.sleep(0.1)

                success = self._preempt_request()
                if not success:
                    break

        # SINGLE: MODE = 'single'
        elif (goal.mode).lower() == 'single' and 0 < goal.sound_id:
            success = self._preempt_request()
            self._play_once_completly(self._median_path, goal.sound_id)

        else:
            rospy.logwarn('[MP] Invalid Request')

        if success:
            self._result = MedianPlayResult()
            self._result.result = self._feedback.feedback.playing_id
            try:
                self._player.stop()
            except:
                pass
            rospy.loginfo('[MP] %s: Finished' % self._action_name)
            self._action_server.set_succeeded(self._result)


def main():
    rospy.init_node('median_player_server')
    MedianPlayerServer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down ... ')


if __name__ == '__main__':
    main()
