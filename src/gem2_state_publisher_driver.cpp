/*
 * gem2_state_publisher - a ros package to generate necessary data for gem2
 *
 * Copyright 2020-2021 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *	 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <gem2_state_publisher/gem2_state_publisher.h>

using std::cerr;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gem2_state_publisher");
    ros::NodeHandle n;
    if (!ros::master::check())
    {
        cerr << "Could not contact master!\nQuitting... " << endl;
        return -1;
    }

    ros::NodeHandle n_p("~");

    gem2_state_publisher *gsp = new gem2_state_publisher();
    gsp->connect(n);
    if (!gsp->connected())
    {
        ROS_ERROR("Could not connect to the Humanoid robot!");
        return -1;
    }

    // Run the spinner in a separate thread to prevent lockups
    gsp->run();

    //Done here
    ROS_INFO("Quitting... ");
    delete gsp;
    return 0;
}
