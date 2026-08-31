#!/usr/bin/env python3
"""Scope every <topic> in a GENERATED robot description with a robot name.

WHY THIS EXISTS. Namespacing the ROS side (an earlier module) is necessary and not
sufficient. cortex_amr.gazebo.xacro writes all 6 of its <topic> elements as bare
names -- cmd_vel, camera, imu, scan_front, scan_rear, joint_states -- which is
correct for one robot and merges two. A ROS namespace has no authority over a
simulator topic, so both models end up on one shared /cmd_vel and the bridge
cannot un-merge them.

We rewrite the GENERATED file, not the source xacro, so a fleet keeps sharing one
description. The better long-term fix is a namespace argument in the xacro that
interpolates into all six at generation time; this is the version you use when
you want two robots running this afternoon.

WHICH FILE YOU FEED IT. Either works and I checked both, because they are not
the same file. `xacro cortex_amr.urdf.xacro` gives you URDF with 21 links; the
SDF I actually spawned was Gazebo's converted form, SDF 1.11 with the fixed
joints collapsed into 3 links. The <topic> elements are identical in both (all 6,
same names) and all three plugins survive the conversion, so scoping either one
gives the same robot. Do not expect the two files to diff clean -- they are 786
and 922 lines and share almost no text.

MEASURED 2026-08-21: 6 elements scoped. Driving /r1/cmd_vel forward while
/r2/cmd_vel went backward moved r1 from x=-3.000 to -0.227 and r2 from 0.400 to
0.585 in the same 14 s. Before scoping, either command drove both.

  usage: scope_sdf.py <in.sdf> <ns> <out.sdf>
"""
import re
import sys


# <topic> is NOT the whole job, and believing it was cost a whole live run.
# The diff-drive and odometry-publisher plugins name their topics and their TF
# FRAMES with different element names, none of which a <topic> regex sees:
#
#   <odom_topic>odom</odom_topic>          both robots publish one /odom
#   <tf_topic>/tf</tf_topic>               both robots publish one /tf
#   <frame_id>odom</frame_id>              both robots' TF says "odom"
#   <child_frame_id>base_footprint</...>   ...to "base_footprint"
#   <odom_frame>truth_odom</odom_frame>    ground-truth odom, same collision
#   <robot_base_frame>base_footprint</...>
#
# Two robots sharing ONE frame name is worse than sharing a topic: the TF tree
# gets two different transforms for the same parent/child pair at the same time,
# and every consumer silently gets whichever arrived last.
EXPECTED = 16     # measured on cortex_amr 2026-08-22 (13 + 3 gz_frame_id)
TOPIC_ELEMENTS = ('topic', 'odom_topic', 'tf_topic')
# gz_frame_id is the frame stamped on SENSOR messages, and it is a FOURTH kind
# of name that needs scoping. Missing it left amcl logging
#   "Message Filter dropping message: frame 'lidar_front_link' ..."
# for every scan, because the laser arrived stamped with the bare link name while
# robot_state_publisher (with frame_prefix) publishes r2/lidar_front_link. The
# scan was flowing and being silently discarded, so amcl never localised and
# planner_server never activated. Measured 2026-08-22 on r2.
FRAME_ELEMENTS = ('frame_id', 'child_frame_id', 'odom_frame',
                  'robot_base_frame', 'gz_frame_id')


def scope(text, ns):
    """Prefix every topic AND frame element with ns.

    Topics keep a leading slash if they had one. Frames never take a leading
    slash: a TF frame id is a bare name, and tf2 rejects one that starts with
    '/' (it strips it and warns, which is worse -- it looks like it worked).
    """
    n = [0]

    def topic_sub(m):
        tag, t = m.group(1), m.group(2)
        n[0] += 1
        if t.startswith('/'):
            return '<%s>/%s%s</%s>' % (tag, ns, t, tag)
        return '<%s>%s/%s</%s>' % (tag, ns, t, tag)

    def frame_sub(m):
        tag, f = m.group(1), m.group(2)
        n[0] += 1
        return '<%s>%s/%s</%s>' % (tag, ns, f.lstrip('/'), tag)

    out = text
    out = re.sub(r'<(%s)>([^<]+)</\1>' % '|'.join(TOPIC_ELEMENTS), topic_sub, out)
    out = re.sub(r'<(%s)>([^<]+)</\1>' % '|'.join(FRAME_ELEMENTS), frame_sub, out)
    return out, n[0]


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__.strip().splitlines()[-1].strip())
    src, ns, dst = sys.argv[1], sys.argv[2], sys.argv[3]
    text = open(src).read()
    pat = r'<(%s)>([^<]+)</\1>' % '|'.join(TOPIC_ELEMENTS + FRAME_ELEMENTS)
    before = re.findall(pat, text)
    out, n = scope(text, ns)
    # A description with nothing to scope cannot be driven or transformed;
    # silently writing it would look like success. Refuse instead.
    if n == 0:
        sys.exit("scope_sdf: %s has no topic or frame elements to scope" % src)
    # And assert the count we measured, so a description that grows a new
    # plugin does not quietly ship half-scoped. Measured 2026-08-22: 6 <topic>
    # + 3 topic-ish + 4 frame elements = 13.
    if n < EXPECTED:
        sys.exit("scope_sdf: scoped only %d of an expected %d elements in %s.\n"
                 "  A new plugin probably added a topic or frame element this "
                 "script does not know about. Grep the generated file for "
                 "'_topic>' and 'frame' and add it to the lists." % (n, EXPECTED, src))
    open(dst, 'w').write(out)
    print("scoped %d topic/frame elements under '%s'" % (n, ns))
    for tag, b in before:
        print("  %-18s %-16s ->  %s/%s" % ('<'+tag+'>', b, ns, b.lstrip('/')))


if __name__ == "__main__":
    main()
