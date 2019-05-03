#!/usr/bin/env python
import rospy
import threading
from flask import abort, Flask, request, jsonify
from rpi_integration_msgs.srv import RobotCommand as command_srv
from rpi_integration_msgs.msg import RobotCommand_message as command_msg
# from std_msgs.msg import String

app  = Flask(__name__)
name = "http_to_srv"

server = rospy.ServiceProxy(name, command_srv)
pub = rospy.Publisher('/flask_node_to_srv', command_msg, queue_size=10)

@app.route("/robotcommand", methods=["POST"])
def getCommand():
    print("Receiving command from OntoSem...")

    print(request.is_json)

    if not request.is_json:
        rospy.logwarn("Bad request!")
        abort(400)

    else:
        cmd = request.get_json()
        # cmd = request.form
        rospy.loginfo(cmd)

        print("RECEIVED:", cmd)

        # format message correctly
        msg_cmd = ""
        for key in cmd:
            if key == "callback":
                # value is callback
                callback = cmd[key]

            else:
                msg_cmd = key
                msg_id = cmd[key]

        if not msg_cmd or not callback:
            rospy.logwarn("No valid command found")
            abort(500)

        else:
            pub.publish(command_msg(msg_cmd, msg_id, callback))

    return "OK"


def flask_callback(msg):
    # msg is of type command_msg: cmd, id, callback

    # publish to server
    # srv.req = command_srv(msg.cmd, msg.id, msg.callback)
    server(msg.cmd, msg.id, msg.callback)
    # server(msg["cmd"], msg["id"], msg["callback"])

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node(name="flask_node", disable_signals=True)).start()

    sub = rospy.Subscriber('/flask_node_to_srv', command_msg, flask_callback)

    rospy.loginfo("Starting {} node".format(name))
    rospy.loginfo("Waiting for OntoSem Cmd service")
    rospy.wait_for_service(name)
    rospy.loginfo("Service ready!")
    print("RUNNING HTTP NODE!")
    app.run(host='localhost', port=7777, debug=True, use_reloader=False)
    rospy.spin()
