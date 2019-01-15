from alpyca.comm import Node


node = Node()


@node.topic_sub('/test_topic/sub', 'std_msgs.msg.String')
def sub_callback(msg):
    print('callback from sub_callback: {0}'.format(msg))

@node.topic_pub('/test_topic/pub', 'std_msgs.msg.String')
def pub_given_data(data):
    return data

@node.topic_pub('/test_topic/pub', 'std_msgs.msg.String')
def pub_data_intern():
    return 'intern random generated data'

@node.service_provide('/test_srv/service', 'alpyca.srv.ExampleSrv')
def srv_prov(data):
    return 'service call with given data included {}'.format(data)

@node.service_call('/test_srv/service', 'alpyca.srv.ExampleSrv')
def srv_call(response):
    return 'service response: {}'.format(response)

@node.start_action_server('/test_act/name', 'alpyca.action.Example')
def act_server(goal):
    for i in range(1, goal.order):
        node.send_action_feedback('{}'.format(i))
    node.set_action_result('114')

def feedback_cb(msg):
    print('Feedback received: {}'.format(msg))   

@node.action_client('/test_act/name', 'alpyca.action.Example', feedback_cb)
def act_client(goal):
    print('client goal: {}'.format(goal))
    return node.get_action_result()

@node.main()
def main():
    pub_given_data('given test data')
    pub_data_intern()
    print('action result: {}'.format(act_client(111)))
    # print(srv_call('service_data'))


if __name__ == "__main__":
    node.run()