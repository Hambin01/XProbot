#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import MySQLdb


class mysqlTask():
    def __init__(self):
        self._use_db = rospy.get_param('use_db', False)
        if self._use_db:
            self._task_info_db = MySQLdb.connect(
                "localhost", "root", "343", "TaskInfoDB", charset='utf8')
            # self._cursor = self._task_info_db.cursor()
            self.mysql_pub = rospy.Publisher('mysql_task_record',
                                             String,
                                             queue_size=100,
                                             latch=True)
            rospy.Subscriber('mysql_cmd',
                             String,
                             self.mysqlCallback,
                             queue_size=1000)

    def mysqlCallback(self, msg):
        sql = msg.data
        rospy.loginfo('[MYSQL] %s' % sql)
        if sql[:6].upper() == "INSERT":
            self.mysql_insert_update_data(sql)
        elif sql[:6].upper() == "UPDATE":
            self.mysql_insert_update_data(sql)
        elif sql[:6].upper() == "SELECT":
            self.mysql_select_data(sql)

    def mysql_insert_update_data(self, sql):
        try:
            self._task_info_db.ping(True)
            cursor = self._task_info_db.cursor()
            cursor.execute(sql)
            self._task_info_db.commit()
            rospy.loginfo('[MYSQL] finished')
        except Exception as e:
            rospy.logwarn("mysql insert: %s" % str(e))
            self._task_info_db.rollback()
        # self.mysql_task_update()


    def mysql_select_data(self, sql):
        try:
            self._task_info_db.ping(True)
            cursor = self._task_info_db.cursor()
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            rospy.logwarn("mysql select: %s" % str(e))
            return ()

    def mysql_task_update(self):
        sql = "SELECT * FROM TaskInfoTable WHERE task_state = '%s' OR task_state = '%s'" % (
            'assigned', 'to assign')
        results = self.mysql_select_data(sql)
        msyql_task_msg = String()
        msyql_task_msg.data = str(results)
        self.mysql_pub.publish(msyql_task_msg)


def main():
    rospy.init_node('mysql_node')
    mysqlTask()
    try:
        rospy.spin()
    except:
        print "Shutting down..."

if __name__ == '__main__':
    main()
