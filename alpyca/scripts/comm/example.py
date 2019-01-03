from alpyca_comm import Node


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

@node.main()
def main():
    pub_given_data('given test data')
    pub_data_intern()


if __name__ == "__main__":
    node.run()