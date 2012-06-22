/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
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
#ifndef VIEW_MANAGER_H
#define VIEW_MANAGER_H

#include <string>

#include <QObject>
#include <QList>

namespace rviz
{

class DisplayContext;
class ViewController;

class ViewManager: public QObject
{
Q_OBJECT
public:
  ViewManager( DisplayContext* context );
  ~ViewManager();

  /**
   * @brief Return the current ViewController in use for the main RenderWindow.
   */
  ViewController* getCurrentViewController() { return view_controller_; }

  /**
   * @brief Return the type of the current ViewController as a
   *        std::string, like "rviz::OrbitViewController".
   */
  std::string getCurrentViewControllerType();

  /**
   * @brief Set the current view controller by specifying the desired type.
   *
   * This accepts the actual C++ class name (with namespace) of the
   * subclass of ViewController and also accepts a number of variants for backward-compatibility:
   *  - "rviz::OrbitViewController", "Orbit"
   *  - "rviz::XYOrbitViewController", "XYOrbit", "rviz::SimpleOrbitViewController", "SimpleOrbit"
   *  - "rviz::FPSViewController", "FPS"
   *  - "rviz::FixedOrientationOrthoViewController", "TopDownOrtho", "Top-down Orthographic"
   *
   * If @a type is not one of these and there is not a current
   * ViewController, the type defaults to rviz::OrbitViewController.
   * If @a type is not one of these and there *is* a current
   * ViewController, nothing happens.
   *
   * If the selected type is different from the current type, a new
   * instance of the selected type is created, set in the main
   * RenderPanel, and sent out via the viewControllerChanged() signal.
   */
  bool setCurrentViewControllerType(const std::string& type);

Q_SIGNALS:
  /**
   * @brief Emitted when a new ViewController type is added.
   * @param class_name is the C++ class name with namespace, like "rviz::OrbitViewController".
   * @param name is the name used for displaying, like "Orbit".
   */
  void viewControllerTypeAdded( const std::string& class_name, const std::string& name );

  /**
   * @brief Emitted after the current ViewController has changed.
   */
  void viewControllerChanged( ViewController* );

private:
  void addViewController(const std::string& class_name, const std::string& name);

  ViewController* view_controller_;
  DisplayContext* context_;
};

} // end namespace rviz

#endif // VIEW_MANAGER_H
