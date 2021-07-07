#include "simulation/kinematics.h"

#include <Eigen/Geometry>
#include <iostream>
#include <queue>
#include "Eigen/Dense"
#include "acclaim/bone.h"
#include "util/helper.h"

using namespace std;
namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    // TODO

    vector<Eigen::Vector4d> rot = posture.bone_rotations;
    vector<Eigen::Vector4d> tran = posture.bone_translations;

    bone->start_position = Eigen::Vector4d::Zero();
    bone->end_position = Eigen::Vector4d::Zero();
    bone->rotation = Eigen::Affine3d::Identity();

    bone->start_position = bone->start_position + tran[bone->idx];
    bone->end_position = bone->start_position + bone->dir * bone->length;
    bone->rotation = Eigen::Affine3d(util::rotateDegreeZYX(rot[bone->idx]));

    // use BFS to traval all bones
    queue<acclaim::Bone*> q;
    if (bone->child != NULL) {
        q.push(bone->child);
        bone = bone->child;
        while (bone->sibling != NULL) {
            q.push(bone->sibling);
            bone = bone->sibling;
        }
    }
    while (!q.empty()) {
        acclaim::Bone* curr = q.front();

        curr->start_position = Eigen::Vector4d::Zero();
        curr->end_position = Eigen::Vector4d::Zero();
        curr->rotation = Eigen::Affine3d::Identity();
        Eigen::Affine3d R = Eigen::Affine3d::Identity();

		// answer:
        R = (curr->rot_parent_current) * (util::rotateDegreeZYX(rot[curr->idx]));
        curr->rotation = curr->parent->rotation * R;
        curr->start_position = curr->parent->end_position;
        curr->end_position = curr->start_position + curr->rotation * (curr->dir * curr->length);
		// end of answer

		// Bonus:
        /*curr->rotation = Eigen::Affine3d(util::rotateDegreeXYZ(rot[curr->idx])) * curr->rot_parent_current;
        curr->start_position = curr->parent->end_position;
        curr->end_position = curr->start_position + curr->rotation * (curr->dir * curr->length);*/
		// end of bonus

        if (curr->child != NULL) {
            q.push(curr->child);
            curr = curr->child;
            while (curr->sibling != NULL) {
                q.push(curr->sibling);
                curr = curr->sibling;
            }
        }
        q.pop();
    }
}

std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int keyframe_old,
                                         int keyframe_new) {
    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures = postures;
    cout << "total_frames = " << total_frames << endl;
    for (int i = 0; i < total_frames; ++i) {
        for (int j = 0; j < total_bones; ++j) {
            // answer
            double key, key1, key2;
            if (i <= keyframe_new)
                key = ((double)keyframe_old) * i / (double)keyframe_new;
            else
                key = ((double)(total_frames - 1 - keyframe_old) * (i - keyframe_new))/(total_frames - 1 - keyframe_new) +
                      keyframe_old;

            if ((int)key == key) {
                new_postures[i].bone_translations[j] = postures[key].bone_translations[j];
                new_postures[i].bone_rotations[j] = postures[key].bone_rotations[j];
                continue;
            }
            key1 = (int)key;
            key2 = (int)key + 1;
            if (key2 >= total_frames) key2 = key1;

			// translation
            double a = key - key1, b = key2 - key;
            new_postures[i].bone_translations[j] =
                (b * postures[key1].bone_translations[j] + a * postures[key2].bone_translations[j]) / (a + b);

            // rotation
            Eigen::Quaterniond q1 = util::rotateDegreeXYZ(postures[key1].bone_rotations[j]);               
            Eigen::Quaterniond q2 = util::rotateDegreeXYZ(postures[key2].bone_rotations[j]);

			Eigen::Quaterniond q_res = q1.slerp(a / (a + b), q2);
            q_res.normalize();

            Eigen::Vector3d tmp1 = q_res.toRotationMatrix().eulerAngles(0, 1, 2);
            Eigen::Vector4d tmp2(tmp1(0), tmp1(1), tmp1(2), 0.0);

			Eigen::Vector4d res = util::toDegree(tmp2);
            new_postures[i].bone_rotations[j] = res;
            // end of answer

			// Discussion: no slerp
            /*double key, key1, key2;
            if (i <= keyframe_new)
                key = ((double)keyframe_old) / keyframe_new * i;
            else
                key = ((double)(total_frames - keyframe_old)) / (total_frames - keyframe_new) * (i - keyframe_new) +
            keyframe_old;

            if ((int)key==key) {
                new_postures[i].bone_translations[j] = postures[key].bone_translations[j];
                new_postures[i].bone_rotations[j] = postures[key].bone_rotations[j];
            } else {
                key1 = (int)key;
                key2 = (int)key + 1;
                double a = key - key1, b = key2 - key;
                new_postures[i].bone_translations[j] =
                    (b * postures[key1].bone_translations[j] + a * postures[key2].bone_translations[j]) / (a + b);
                new_postures[i].bone_rotations[j] = postures[key1].bone_rotations[j];
            }
            */
            // end of no slerp
        }
    }
    return new_postures;
}
}  // namespace kinematics