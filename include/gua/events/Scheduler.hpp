/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef SCHEDULER_HPP_
#define SCHEDULER_HPP_

#include <functional>
#include <map>

#include <boost/asio.hpp>

#include <gua/platform.hpp>
#include <gua/events/MainLoop.hpp>

namespace gua {
namespace events {

class GUA_DLL Scheduler {
 public:
  Scheduler();
  ~Scheduler();

  void execute_delayed(MainLoop& loop, std::function<void()> callback, double delay);

 private:

   void self_callback(boost::asio::deadline_timer* timer, int revents);

   std::map<boost::asio::deadline_timer*, std::function<void()> > tasks_;

};

}
}

#endif /* SCHEDULER_HPP_ */
