/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#ifndef TOPIC_HANDLE_H
#define TOPIC_HANDLE_H

#include <gripper_ros_common/VelocitySetpoint.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <bhand_mux/bhand_mux.h>
#include <bhand_mux/utils.h>

#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>

#include <string>
#include <vector>

namespace bhand_mux {

template <typename T> class TopicHandle_ : boost::noncopyable {
  public:
    typedef int priority_type;

    /**
     * @brief TopicHandle_
     * @param nh Node handle
     * @param name Name identifier
     * @param topic Topic name
     * @param timeout Timeout to consider that the messages are old; note
     * that initially the message stamp is set to 0.0, so the message has
     * expired
     * @param priority Priority of the topic
     */
    TopicHandle_(ros::NodeHandle &nh, const std::string &name, const std::string &topic, double timeout,
                 priority_type priority, BhandMux *mux)
        : nh_(nh), name_(name), topic_(topic), timeout_(timeout),
          priority_(clamp(priority, priority_type(0), priority_type(255))), mux_(mux), stamp_(0.0) {
        ROS_INFO_STREAM("Topic handler '" << name_ << "' subscribed to topic '" << topic_
                                          << "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None")
                                          << ", priority = " << static_cast<int>(priority_));
    }

    virtual ~TopicHandle_() { subscriber_.shutdown(); }

    /**
     * @brief hasExpired
     * @return true if the message has expired; false otherwise.
     *         If the timeout is set to 0.0, this function always returns
     *         false
     */
    bool hasExpired() const { return (timeout_ > 0.0) && ((ros::Time::now() - stamp_).toSec() > timeout_); }

    const std::string &getName() const { return name_; }

    const std::string &getTopic() const { return topic_; }

    const double &getTimeout() const { return timeout_; }

    /**
     * @brief getPriority Priority getter
     * @return Priority
     */
    const priority_type &getPriority() const { return priority_; }

    const T &getStamp() const { return stamp_; }

    const T &getMessage() const { return msg_; }

  protected:
    ros::NodeHandle nh_;

    std::string name_;
    std::string topic_;
    ros::Subscriber subscriber_;
    double timeout_;
    priority_type priority_;

  protected:
    BhandMux *mux_;

    ros::Time stamp_;
    T msg_;
};

class VelocityTopicHandle : public TopicHandle_<gripper_ros_common::VelocitySetpoint> {
  private:
    typedef TopicHandle_<gripper_ros_common::VelocitySetpoint> base_type;

  public:
    typedef typename base_type::priority_type priority_type;

    VelocityTopicHandle(ros::NodeHandle &nh, const std::string &name, const std::string &topic, double timeout,
                        priority_type priority, BhandMux *mux)
        : base_type(nh, name, topic, timeout, priority, mux) {
        subscriber_ = nh_.subscribe(topic_, 1, &VelocityTopicHandle::callback, this);
    }

    bool isMasked(priority_type lock_priority) const { return hasExpired() || (getPriority() < lock_priority); }

    void callback(const gripper_ros_common::VelocitySetpointConstPtr &msg) {
        stamp_ = ros::Time::now();
        msg_ = *msg;

        // Check if this twist has priority.
        // Note that we have to check all the locks because they might time out
        // and since we have several topics we must look for the highest one in
        // all the topic list; so far there's no O(1) solution.
        if (mux_->hasPriority(*this)) {
            mux_->publishBhand(msg);
        }
    }
};

class LockTopicHandle : public TopicHandle_<std_msgs::Bool> {
  private:
    typedef TopicHandle_<std_msgs::Bool> base_type;

  public:
    typedef typename base_type::priority_type priority_type;

    LockTopicHandle(ros::NodeHandle &nh, const std::string &name, const std::string &topic, double timeout,
                    priority_type priority, BhandMux *mux)
        : base_type(nh, name, topic, timeout, priority, mux) {
        subscriber_ = nh_.subscribe(topic_, 1, &LockTopicHandle::callback, this);
    }

    /**
     * @brief isLocked
     * @return true if has expired or locked (i.e. bool message data is true)
     */
    bool isLocked() const { return hasExpired() || getMessage().data; }

    void callback(const std_msgs::BoolConstPtr &msg) {
        stamp_ = ros::Time::now();
        msg_ = *msg;
    }
};

} // namespace bhand_mux

#endif // TOPIC_HANDLE_H
