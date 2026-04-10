/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/scenarios/lane_follow_park/lane_follow_park_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;
using apollo::cyber::Clock;

namespace {
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

bool LaneFollowParkStage::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    CHECK_NOTNULL(context);
    bool ret = Stage::Init(config, injector, config_dir, context);
    if (!ret) {
        AERROR << Name() << "init failed!";
        return false;
    }
    scenario_config_.CopyFrom(GetContextAs<LaneFollowParkContext>()->scenario_config);
    return ret;
}

void LaneFollowParkStage::RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info) {
    if (!FLAGS_enable_record_debug) {
        ADEBUG << "Skip record debug info";
        return;
    }
    auto ptr_debug = reference_line_info->mutable_debug();

    const auto path_decision = reference_line_info->path_decision();
    for (const auto obstacle : path_decision->obstacles().Items()) {
        auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
        obstacle_debug->set_id(obstacle->Id());
        obstacle_debug->mutable_sl_boundary()->CopyFrom(obstacle->PerceptionSLBoundary());
        const auto& decider_tags = obstacle->decider_tags();
        const auto& decisions = obstacle->decisions();
        if (decider_tags.size() != decisions.size()) {
            AERROR << "decider_tags size: " << decider_tags.size()
                   << " different from decisions size:" << decisions.size();
        }
        for (size_t i = 0; i < decider_tags.size(); ++i) {
            auto decision_tag = obstacle_debug->add_decision_tag();
            decision_tag->set_decider_tag(decider_tags[i]);
            decision_tag->mutable_decision()->CopyFrom(decisions[i]);
        }
    }
}

StageResult LaneFollowParkStage::Process(const TrajectoryPoint& planning_start_point, Frame* frame) {
    if (frame->reference_line_info().empty()) {
        return StageResult(StageStatusType::FINISHED);
    }

    bool has_drivable_reference_line = false;

    ADEBUG << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();

    unsigned int count = 0;
    StageResult result;
    int need_escape = 0;
    for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
        // TODO(SHU): need refactor
        if (count++ == frame->mutable_reference_line_info()->size()) {
            break;
        }
        ADEBUG << "No: [" << count << "] Reference Line.";
        ADEBUG << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath();

        if (has_drivable_reference_line) {
            reference_line_info.SetDrivable(false);
            break;
        }

        result = PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

        bool escape = IsNeedEscape(reference_line_info);
        need_escape = (escape && need_escape != -1) ? 1 : -1;
        AINFO << "need escape: " << need_escape;

        if (!result.HasError()) {
            if (!reference_line_info.IsChangeLanePath()) {
                ADEBUG << "reference line is NOT lane change ref.";
                has_drivable_reference_line = true;
                continue;
            }
            if (reference_line_info.Cost() < kStraightForwardLineCost) {
                // If the path and speed optimization succeed on target lane while
                // under smart lane-change or IsClearToChangeLane under older version
                has_drivable_reference_line = true;
                reference_line_info.SetDrivable(true);
            } else {
                reference_line_info.SetDrivable(false);
                ADEBUG << "\tlane change failed";
            }
        } else {
            reference_line_info.SetDrivable(false);
        }
    }
    AINFO << "need escape: " << need_escape;
    if (need_escape == 1) {
        next_stage_ = "LANE_ESCAPE_PARK_STAGE";
        return StageResult(StageStatusType::FINISHED);
    }

    return has_drivable_reference_line ? result.SetStageStatus(StageStatusType::RUNNING)
                                       : result.SetStageStatus(StageStatusType::ERROR);
}

StageResult LaneFollowParkStage::PlanOnReferenceLine(
        const TrajectoryPoint& planning_start_point,
        Frame* frame,
        ReferenceLineInfo* reference_line_info) {
    if (!reference_line_info->IsChangeLanePath()) {
        reference_line_info->AddCost(kStraightForwardLineCost);
    }
    ADEBUG << "planning start point:" << planning_start_point.DebugString();
    ADEBUG << "Current reference_line_info is IsChangeLanePath: " << reference_line_info->IsChangeLanePath();

    StageResult ret;
    for (auto task : task_list_) {
        const double start_timestamp = Clock::NowInSeconds();
        const auto start_planning_perf_timestamp
                = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        ret.SetTaskStatus(task->Execute(frame, reference_line_info));

        const double end_timestamp = Clock::NowInSeconds();
        const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
        ADEBUG << "after task[" << task->Name() << "]:" << reference_line_info->PathSpeedDebugString();
        ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
        RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);

        const auto end_planning_perf_timestamp
                = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto plnning_perf_ms = (end_planning_perf_timestamp - start_planning_perf_timestamp) * 1000;
        AINFO << "Planning Perf: task name [" << task->Name() << "], " << plnning_perf_ms << " ms.";

        if (ret.IsTaskError()) {
            AERROR << "Failed to run tasks[" << task->Name()
                   << "], Error message: " << ret.GetTaskStatus().error_message();
            break;
        }

        // TODO(SHU): disable reference line order changes for now
        // updated reference_line_info, because it is changed in
        // lane_change_decider by PrioritizeChangeLane().
        // reference_line_info = &frame->mutable_reference_line_info()->front();
        // ADEBUG << "Current reference_line_info is IsChangeLanePath: "
        //        << reference_line_info->IsChangeLanePath();
    }

    RecordObstacleDebugInfo(reference_line_info);

    // check path and speed results for path or speed fallback
    reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
    if (ret.IsTaskError()) {
        fallback_task_->Execute(frame, reference_line_info);
    }

    DiscretizedTrajectory trajectory;
    if (!reference_line_info->CombinePathAndSpeedProfile(
                planning_start_point.relative_time(), planning_start_point.path_point().s(), &trajectory)) {
        const std::string msg = "Fail to aggregate planning trajectory.";
        AERROR << msg;
        return ret.SetStageStatus(StageStatusType::ERROR, msg);
    }

    // determine if there is a destination on reference line.
    double dest_stop_s = -1.0;
    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle->LongitudinalDecision().has_stop()
            && obstacle->LongitudinalDecision().stop().reason_code() == STOP_REASON_DESTINATION) {
            SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(), reference_line_info->reference_line());
            dest_stop_s = dest_sl.s();
        }
    }

    for (const auto* obstacle : reference_line_info->path_decision()->obstacles().Items()) {
        if (obstacle->IsVirtual()) {
            continue;
        }
        if (!obstacle->IsStatic()) {
            continue;
        }
        if (obstacle->LongitudinalDecision().has_stop()) {
            bool add_stop_obstacle_cost = false;
            if (dest_stop_s < 0.0) {
                add_stop_obstacle_cost = true;
            } else {
                SLPoint stop_sl
                        = GetStopSL(obstacle->LongitudinalDecision().stop(), reference_line_info->reference_line());
                if (stop_sl.s() < dest_stop_s && (dest_stop_s - reference_line_info->AdcSlBoundary().end_s()) < 20.0) {
                    add_stop_obstacle_cost = true;
                }
            }
            if (add_stop_obstacle_cost) {
                static constexpr double kReferenceLineStaticObsCost = 1e3;
                reference_line_info->AddCost(kReferenceLineStaticObsCost);
            }
        }
    }

    if (FLAGS_enable_trajectory_check) {
        if (ConstraintChecker::ValidTrajectory(trajectory) != ConstraintChecker::Result::VALID) {
            const std::string msg = "Current planning trajectory is not valid.";
            AERROR << msg;
            return ret.SetStageStatus(StageStatusType::ERROR, msg);
        }
    }

    reference_line_info->SetTrajectory(trajectory);
    reference_line_info->SetDrivable(true);
    ret.SetStageStatus(StageStatusType::RUNNING);
    return ret;
}

SLPoint LaneFollowParkStage::GetStopSL(const ObjectStop& stop_decision, const ReferenceLine& reference_line) const {
    SLPoint sl_point;
    reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
    return sl_point;
}

bool LaneFollowParkStage::IsNeedEscape(const ReferenceLineInfo& reference_line_info) {
    AINFO << "IsNeedEscape";
    if (std::fabs(reference_line_info.vehicle_state().linear_velocity())
        > common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped()) {
        AINFO << "Vehicle is not stopped.";
        return false;
    }

    if (reference_line_info.path_data().Empty()) {
        AINFO << "reference_line_info.path_data() is empty.";
        return false;
    }

    // if (reference_line_info.path_data().path_label().find("regular") != std::string::npos &&
    //     reference_line_info.path_data().blocking_obstacle_id().empty()) {
    //   AINFO << "find regular path: " << reference_line_info.path_data().path_label();
    //   return false;
    // }

    if (reference_line_info.SDistanceToDestination() < 20.0) {
        AINFO << "Distance to destination is less than 20m.";
        return false;
    }

    double min_distance = std::numeric_limits<double>::max();
    std::string stop_decision_obs_id;
    std::string block_obs_id;
    GetClosestStopDecisionObs(reference_line_info, stop_decision_obs_id, min_distance);
    AINFO << "min_distance: " << min_distance;
    if (min_distance < FLAGS_min_stop_distance_obstacle + 2.0 && min_distance > 0) {
        block_obs_id = stop_decision_obs_id;
        AINFO << "block_obs_id: " << block_obs_id;
    } else {
        queue_sence_obs_info_.first.reset();
        queue_sence_obs_info_.second = 0;
        stable_block_obs_count_.first.clear();
        stable_block_obs_count_.second = 0;
        return false;
    }

    double dis = DistanceBlockingObstacleToJunction(reference_line_info, block_obs_id);
    AINFO << "DistanceBlockingObstacleToIntersection: " << dis;

    if (DistanceBlockingObstacleToJunction(reference_line_info, block_obs_id)
        < scenario_config_.min_distance_block_obs_to_junction()) {
        AINFO << "DistanceBlockingObstacleToIntersection is smaller than 22m.";
        return false;
    }
    if (!IsEnoughSpace(reference_line_info, block_obs_id)) {
        AINFO << "IsEnoughSpace: false";
        return false;
    }

    bool is_queue = IsQueueSence(reference_line_info, block_obs_id);
    bool is_stable_block_obs = IsStableBlockObs(block_obs_id);

    if (is_queue || !is_stable_block_obs) {
        AINFO << "IsNeedEscape: false";
        return false;
    }

    return true;
}

bool LaneFollowParkStage::IsEnoughSpace(
        const ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id) {
    bool enable_lane_borrow = false;
    bool lane_follow_enough_space = false;
    double check_s = reference_line_info.AdcSlBoundary().end_s();
    auto ref_point = reference_line_info.reference_line().GetNearestReferencePoint(check_s);
    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
    auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
    if (ptr_lane_info == nullptr) {
        AWARN << "ptr_lane_info is null";
        return false;
    }
    if (!ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()
        || !ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
        if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
            AINFO << "cannot borrow left lane";
        } else {
            enable_lane_borrow = true;
        }
    }
    if (!ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()
        || !ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        lane_boundary_type = hdmap::RightBoundaryType(waypoint);
        if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW
            || lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
            AINFO << "cannot borrow right lane";
        } else {
            enable_lane_borrow = true;
        }
    }

    const Obstacle* block_obs = reference_line_info.path_decision().Find(blocking_obstacle_id);
    double ego_width = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().width();
    const auto obstacle_sl = block_obs->PerceptionSLBoundary();
    double obs_start_l = obstacle_sl.start_l();
    double obs_end_l = obstacle_sl.end_l();
    double check_start_s = obstacle_sl.start_s();
    double check_end_s = obstacle_sl.end_s();
    double obs_s = (check_start_s + check_end_s) / 2;
    double left_lane_width = 0.0;
    double right_lane_width = 0.0;
    reference_line_info.reference_line().GetLaneWidth(obs_s, &left_lane_width, &right_lane_width);
    PathDecision path_decision = reference_line_info.path_decision();
    // 0:start of obs l; 1:end of obs l
    std::vector<std::pair<double, int>> obs_l_range;
    for (const auto* ptr_obstacle_item : path_decision.obstacles().Items()) {
        Obstacle* ptr_obstacle = path_decision.Find(ptr_obstacle_item->Id());
        if (ptr_obstacle == nullptr || !ptr_obstacle->is_path_st_boundary_initialized() || !ptr_obstacle->IsStatic()
            || ptr_obstacle->IsVirtual()) {
            continue;
        }
        const SLBoundary& obs_sl_boundary = ptr_obstacle_item->PerceptionSLBoundary();
        if (obs_sl_boundary.start_s() > check_end_s || obs_sl_boundary.end_s() < check_start_s) {
            continue;
        }
        obs_l_range.emplace_back(std::make_pair(obs_sl_boundary.start_l(), 0));
        obs_l_range.emplace_back(std::make_pair(obs_sl_boundary.end_l(), 1));
        AINFO << "obstacle: " << ptr_obstacle->Id() << " " << obs_sl_boundary.start_l() << " "
              << obs_sl_boundary.end_l();
        AINFO << ptr_obstacle->DebugString();
    }
    std::sort(obs_l_range.begin(), obs_l_range.end(), [](std::pair<double, int>& p1, std::pair<double, int>& p2) {
        return p1.first < p2.first;
    });
    int gap_count = 0;
    std::vector<double> space_gap;
    for (size_t i = 0; i < obs_l_range.size(); ++i) {
        if (gap_count == 0) {
            space_gap.push_back(obs_l_range.at(i).first);
            AINFO << "space_gap: " << space_gap.back();
        }
        if (gap_count < 0) {
            AERROR << "check error";
            return false;
        }
        if (obs_l_range.at(i).second == 0) {
            gap_count++;
        } else if (obs_l_range.at(i).second == 1) {
            gap_count--;
        }
        if (gap_count == 0) {
            space_gap.push_back(obs_l_range.at(i).first);
            AINFO << "space_gap: " << space_gap.back();
        }
    }
    if (space_gap.empty()) {
        AINFO << "space_gap is empty";
        space_gap.push_back(left_lane_width);
    }
    double max_gap = right_lane_width + space_gap.front();
    AINFO << "max_gap: " << max_gap;
    for (int i = 2; i < space_gap.size(); i += 2) {
        double temp_gap = space_gap[i] - space_gap[i - 1];
        if (temp_gap > max_gap) {
            max_gap = temp_gap;
            AINFO << "max_gap: " << max_gap;
        }
    }
    max_gap = std::max(max_gap, left_lane_width - space_gap.back());
    AINFO << "max_gap: " << max_gap;
    if (max_gap - ego_width > scenario_config_.passby_min_gap()) {
        lane_follow_enough_space = true;
    } else {
        AINFO << "lane_follow_enough_space: false, max_gap: " << max_gap << " ego_width: " << ego_width;
    }

    if (enable_lane_borrow || lane_follow_enough_space) {
        return true;
    } else {
        AINFO << "enable_lane_borrow: " << enable_lane_borrow
              << ", lane_follow_enough_space: " << lane_follow_enough_space;
        return false;
    }
}

bool LaneFollowParkStage::IsQueueSence(
        const ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id) {
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* obs = reference_line_info.path_decision().obstacles().Find(blocking_obstacle_id);
    if (obs == nullptr) {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
    }

    if (obs->IsVirtual() || obs->PerceptionSLBoundary().start_s() < 0
        || obs->Perception().type() != perception::PerceptionObstacle::VEHICLE) {
        return false;
    }
    double obs_center_l = (obs->PerceptionSLBoundary().start_l() + obs->PerceptionSLBoundary().end_l()) / 2;
    if (obs_center_l < -1.5) {
        AINFO << "side parking car: " << obs->Id();
        return false;
    }

    if (!obs->IsStatic()) {
        AINFO << "moving obs: " << obs->Id();
        return true;
    }

    if (queue_sence_obs_info_.first == nullptr) {
        queue_sence_obs_info_.first = std::make_shared<Obstacle>(*obs);
        queue_sence_obs_info_.second = 1;
        return true;
    }

    if (queue_sence_obs_info_.first->Id() == obs->Id()) {
        if (queue_sence_obs_info_.second < scenario_config_.queue_check_count()) {
            queue_sence_obs_info_.second++;
            AINFO << "obs id: " << obs->Id() << ", queue sence is true.";
            return true;
        } else {
            // block obs static more than 10s
            AINFO << "obs id: " << obs->Id() << ", queue sence is false.";
            return false;
        }
    } else {
        queue_sence_obs_info_.first = std::make_shared<Obstacle>(*obs);
        queue_sence_obs_info_.second = 1;
        AINFO << "obs id: " << obs->Id() << ", queue sence is true.";
        return true;
    }
}

bool LaneFollowParkStage::IsStableBlockObs(const std::string& blocking_obstacle_id) {
    if (blocking_obstacle_id.empty()) {
        ADEBUG << "There is no blocking obstacle.";
        return false;
    }
    if (stable_block_obs_count_.first.empty()) {
        stable_block_obs_count_.first = blocking_obstacle_id;
        stable_block_obs_count_.second = 1;
        return false;
    }

    if (stable_block_obs_count_.first == blocking_obstacle_id) {
        if (stable_block_obs_count_.second < scenario_config_.stable_block_count()) {
            stable_block_obs_count_.second++;
            AINFO << "obs id: " << blocking_obstacle_id << ", stable is false.";
            return false;
        } else {
            // block obs static more than 3s
            AINFO << "obs id: " << blocking_obstacle_id << ", stable is true.";
            return true;
        }
    } else {
        stable_block_obs_count_.first = blocking_obstacle_id;
        stable_block_obs_count_.second = 1;
        AINFO << "obs id: " << blocking_obstacle_id << ", stable is false.";
        return false;
    }
}

void LaneFollowParkStage::GetClosestStopDecisionObs(
        const ReferenceLineInfo& reference_line_info,
        std::string& stop_decision_obs_id,
        double& min_distance) {
    for (const auto& obs : reference_line_info.path_decision().obstacles().Items()) {
        const Obstacle* ptr_obstacle = reference_line_info.path_decision().Find(obs->Id());
        ACHECK(ptr_obstacle != nullptr);
        double dist = ptr_obstacle->PerceptionSLBoundary().start_s() - reference_line_info.AdcSlBoundary().end_s();
        if (!ptr_obstacle->IsVirtual() && ptr_obstacle->LongitudinalDecision().has_stop() && min_distance > dist) {
            min_distance = dist;
            stop_decision_obs_id = ptr_obstacle->Id();
        }
    }
}

}  // namespace planning
}  // namespace apollo
