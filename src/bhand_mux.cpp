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

#include "joy_server/BhandTeleop.h"
#include <bhand_mux/bhand_mux.h>
#include <bhand_mux/bhand_mux_diagnostics.h>
#include <bhand_mux/bhand_mux_diagnostics_status.h>
#include <bhand_mux/topic_handle.h>
#include <bhand_mux/utils.h>
#include <bhand_mux/xmlrpc_helpers.h>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_bhand Old velocity
 * @param new_bhand New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const joy_server::BhandTeleop &old_bhand, const joy_server::BhandTeleop &new_bhand) {
    const auto old_spread = std::abs(old_bhand.spread);
    const auto new_spread = std::abs(new_bhand.spread);

    const auto old_grasp = std::abs(old_bhand.grasp);
    const auto new_grasp = std::abs(new_bhand.grasp);

    return (old_spread < new_spread) || (old_grasp < new_grasp);
}

namespace bhand_mux {

BhandMux::BhandMux(int window_size) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    /// Get topics and locks:
    velocity_hs_ = boost::make_shared<velocity_topic_container>();
    lock_hs_ = boost::make_shared<lock_topic_container>();
    getTopicHandles(nh, nh_priv, "topics", *velocity_hs_);
    getTopicHandles(nh, nh_priv, "locks", *lock_hs_);

    /// Publisher for output topic:
    cmd_pub_ = nh.advertise<joy_server::BhandTeleop>("cmd_vel_out", 1);

    /// Diagnostics:
    diagnostics_ = boost::make_shared<diagnostics_type>();
    status_ = boost::make_shared<status_type>();
    status_->velocity_hs = velocity_hs_;
    status_->lock_hs = lock_hs_;

    diagnostics_timer_ = nh.createTimer(ros::Duration(DIAGNOSTICS_PERIOD), &BhandMux::updateDiagnostics, this);
}

BhandMux::~BhandMux() {}

void BhandMux::updateDiagnostics(const ros::TimerEvent &event) {
    status_->priority = getLockPriority();
    diagnostics_->updateStatus(status_);
}

void BhandMux::publishBhand(const joy_server::BhandTeleopConstPtr &msg) { cmd_pub_.publish(*msg); }

template <typename T>
void BhandMux::getTopicHandles(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, const std::string &param_name,
                               std::list<T> &topic_hs) {
    try {
        xh::Array output;
        xh::fetchParam(nh_priv, param_name, output);

        xh::Struct output_i;
        std::string name, topic;
        double timeout;
        int priority;
        for (int i = 0; i < output.size(); ++i) {
            xh::getArrayItem(output, i, output_i);

            xh::getStructMember(output_i, "name", name);
            xh::getStructMember(output_i, "topic", topic);
            xh::getStructMember(output_i, "timeout", timeout);
            xh::getStructMember(output_i, "priority", priority);

            topic_hs.emplace_back(nh, name, topic, timeout, priority, this);
        }
    } catch (const xh::XmlrpcHelperException &e) {
        ROS_FATAL_STREAM("Error parsing params: " << e.what());
    }
}

int BhandMux::getLockPriority() {
    LockTopicHandle::priority_type priority = 0;

    /// max_element on the priority of lock topic handles satisfying
    /// that is locked:
    for (const auto &lock_h : *lock_hs_) {
        if (lock_h.isLocked()) {
            auto tmp = lock_h.getPriority();
            if (priority < tmp) {
                priority = tmp;
            }
        }
    }

    ROS_DEBUG_STREAM("Priority = " << static_cast<int>(priority));

    return priority;
}

bool BhandMux::hasPriority(const VelocityTopicHandle &bhand) {
    const auto lock_priority = getLockPriority();

    LockTopicHandle::priority_type priority = 0;
    std::string velocity_name = "NULL";

    /// max_element on the priority of velocity topic handles satisfying
    /// that is NOT masked by the lock priority:
    for (const auto &velocity_h : *velocity_hs_) {
        if (!velocity_h.isMasked(lock_priority)) {
            const auto velocity_priority = velocity_h.getPriority();
            if (priority < velocity_priority) {
                priority = velocity_priority;
                velocity_name = velocity_h.getName();
            }
        }
    }

    return bhand.getName() == velocity_name;
}

} // namespace bhand_mux
