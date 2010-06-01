# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)

import os
import shutil
import sys
import threading
import time

import numpy
import wx
import wx.lib.masked
import wx.lib.wxcairo

import Image

from rxbag import bag_helper, TopicMessageView
import image_helper

class ImageView(TopicMessageView):
    name = 'Image'
    
    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)
        
        self._image_lock  = threading.RLock()
        self._image       = None
        self._image_topic = None
        self._image_stamp = None

        self._image_surface = None

        self._overlay_font_size = 14.0
        self._overlay_indent    = (4, 4)
        self._overlay_color     = (0.2, 0.2, 1.0)

        self._size_set = False
        
        tb = self.parent.GetToolBar()
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'
        tb.AddSeparator()
        tb.Bind(wx.EVT_TOOL, lambda e: self.save_frame(),           tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'picture_save.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.save_selected_frames(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'pictures_save.png')))
        
        self.parent.Bind(wx.EVT_SIZE,       self.on_size)
        self.parent.Bind(wx.EVT_PAINT,      self.on_paint)
        self.parent.Bind(wx.EVT_RIGHT_DOWN, self.on_right_down)

    ## MessageView implementation
    
    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        
        topic, msg, t = msg_details[:3]

        if not msg:
            self.set_image(None, topic, stamp)
        else:
            self.set_image(image_helper.imgmsg_to_pil(msg), topic, msg.header.stamp)
    
            if not self._size_set:
                self._size_set = True
                wx.CallAfter(self.reset_size)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

        self.set_image(None, None, None)

    ##
    
    def set_image(self, image, image_topic, image_stamp):
        with self._image_lock:
            self._image         = image
            self._image_surface = None
            
            self._image_topic = image_topic
            self._image_stamp = image_stamp

        wx.CallAfter(self.parent.Refresh)

    def reset_size(self):
        with self._image_lock:
            if self._image:
                self.parent.ClientSize = self._image.size

    ## Events

    def on_size(self, event):
        with self._image_lock:
            self._image_surface = None
            
        self.parent.Refresh()

    def on_paint(self, event):
        dc = wx.lib.wxcairo.ContextFromDC(wx.PaintDC(self.parent))
        
        with self._image_lock:
            if not self._image:
                return

            ix, iy, iw, ih = 0, 0, self.parent.ClientSize[0], self.parent.ClientSize[1]

            # Rescale the bitmap if necessary
            if not self._image_surface:
                if self._image.size[0] != iw or self._image.size[1] != ih:
                    self._image_surface = image_helper.pil_to_cairo(self._image.resize((iw, ih), Image.NEAREST))
                else:
                    self._image_surface = image_helper.pil_to_cairo(self._image)
    
            # Draw bitmap
            dc.set_source_surface(self._image_surface, ix, iy)
            dc.rectangle(ix, iy, iw, ih)
            dc.fill()
    
            # Draw overlay
            dc.set_font_size(self._overlay_font_size)
            font_height = dc.font_extents()[2]
            dc.set_source_rgb(*self._overlay_color)
            dc.move_to(self._overlay_indent[0], self._overlay_indent[1] + font_height)
            dc.show_text(bag_helper.stamp_to_str(self._image_stamp))

    def on_right_down(self, event):
        if self._image:
            self.parent.PopupMenu(ImagePopupMenu(self.parent, self), event.GetPosition())

    ##

    def save_frame(self):
        if not self._image:
            return

        dialog = wx.FileDialog(self.parent.Parent, 'Save frame to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            with self._image_lock:
                if self._image:
                    self._image.save(dialog.Path, 'png')
        dialog.Destroy()

    def save_selected_frames(self):
        if not self._image:
            return
        
        dialog = ExportFramesDialog(self.parent.Parent, -1, 'Export frames', self.timeline, self._image_topic)
        dialog.ShowModal()
        dialog.Destroy()

    def export_video(self):
        bag_file = self.timeline.bag_file

        msg_positions = bag_index.msg_positions[self._image_topic]
        if len(msg_positions) == 0:
            return
        
        dialog = wx.FileDialog(self.parent.Parent, 'Save video to...', wildcard='AVI files (*.avi)|*.avi', style=wx.FD_SAVE)
        if dialog.ShowModal() != wx.ID_OK:
            return
        video_filename = dialog.Path

        import tempfile
        tmpdir = tempfile.mkdtemp()

        frame_count = 0
        w, h = None, None
        
        total_frames = len(bag_file.read_messages(self._image_topic, raw=True))

        for i, (topic, msg, t) in enumerate(bag_file.read_messages(self._image_topic)):
            img = image_helper.imgmsg_to_wx(msg)
            if img:
                frame_filename = '%s/frame-%s.png' % (tmpdir, str(t))
                print '[%d / %d]' % (i + 1, total_frames)
                img.SaveFile(frame_filename, wx.BITMAP_TYPE_PNG)
                frame_count += 1

                if w is None:
                    w, h = img.GetWidth(), img.GetHeight()

        if frame_count > 0:
            positions = numpy.array([stamp for (stamp, pos) in msg_positions])
            if len(positions) > 1:
                spacing = positions[1:] - positions[:-1]
                fps = 1.0 / numpy.median(spacing)

            print 'Encoding %dx%d at %d fps' % (w, h, fps)

            try:
                command = ('mencoder',
                           'mf://' + tmpdir + '/*.png',
                           '-mf',
                           'type=png:w=%d:h=%d:fps=%d' % (w, h, fps),
                           '-ovc',
                           'lavc',
                           '-lavcopts',
                           'vcodec=mpeg4',
                           '-oac',
                           'copy',
                           '-o',
                           video_filename)
                os.spawnvp(os.P_WAIT, 'mencoder', command)
            finally:
                shutil.rmtree(tmpdir)

        dialog.Destroy()

class ImagePopupMenu(wx.Menu):
    def __init__(self, parent, image_view):
        wx.Menu.__init__(self)

        self.image_view = image_view

        # Reset Size
        reset_item = wx.MenuItem(self, wx.NewId(), 'Reset Size')
        self.AppendItem(reset_item)
        self.Bind(wx.EVT_MENU, lambda e: self.image_view.reset_size(), id=reset_item.Id)

        # Save Frame...
        save_frame_item = wx.MenuItem(self, wx.NewId(), 'Save Frame...')
        self.AppendItem(save_frame_item)
        self.Bind(wx.EVT_MENU, lambda e: self.image_view.export_frame(), id=save_frame_item.Id)

        # Save Selected Frames...
        save_selected_frames_item = wx.MenuItem(self, wx.NewId(), 'Save Selected Frames...')
        self.AppendItem(save_selected_frames_item)
        self.Bind(wx.EVT_MENU, lambda e: self.image_view.save_selected_frames(), id=save_selected_frames_item.Id)
        if not self.image_view.timeline.has_selected_region:
            save_selected_frames_item.Enable(False)

        # Export to AVI...
        #export_video_item = wx.MenuItem(self, wx.NewId(), 'Export to AVI...')
        #self.AppendItem(export_video_item)
        #self.Bind(wx.EVT_MENU, lambda e: self.image_view.export_video(), id=export_video_item.Id)

class ExportFramesDialog(wx.Dialog):
    def __init__(self, parent, id, title, timeline, topic):
        wx.Dialog.__init__(self, parent, id, title, size=(280, 90))

        self.timeline = timeline
        self.topic    = topic

        panel = wx.Panel(self, -1)

        self.file_spec_label = wx.StaticText(panel, -1, 'File spec:',    (5, 10))
        self.file_spec_text  = wx.TextCtrl  (panel, -1, 'frame%04d.png', (65, 7), (210, 22))

        self.steps_label = wx.StaticText(panel, -1, 'Every:', (5, 35))
        self.steps_text  = wx.lib.masked.NumCtrl(panel, -1, pos=(65, 32), size=(34, 22), value=1, integerWidth=3, allowNegative=False)
        self.steps_text.SetMin(1)

        self.frames_label = wx.StaticText(panel, -1, 'frame(s)', (102, 36))

        self.export_button = wx.Button(self, -1, 'Export', size=(68, 26))
        self.cancel_button = wx.Button(self, -1, 'Cancel', size=(68, 26))

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(self.export_button, 1)
        hbox.Add(self.cancel_button, 1, wx.LEFT, 5)

        vbox = wx.BoxSizer(wx.VERTICAL)
        vbox.Add(panel)
        vbox.Add(hbox, 1, wx.ALIGN_CENTER | wx.TOP | wx.BOTTOM, 5)
        self.SetSizer(vbox)

        self.Bind(wx.EVT_BUTTON, self._on_export, id=self.export_button.Id)
        self.Bind(wx.EVT_BUTTON, self._on_cancel, id=self.cancel_button.Id)

        self.progress      = None
        self.export_thread = None

    def _on_export(self, event):
        file_spec = self.file_spec_text.Value
        try:
            test_filename = file_spec % 0
        except Exception:
            wx.MessageDialog(None, 'Error with filename specification.\n\nPlease include a frame number format, e.g. frame%04d.png', 'Error', wx.OK | wx.ICON_ERROR).ShowModal()
            return

        try:
            step = max(1, int(self.steps_text.Value))
        except ValueError:
            step = 1

        bag_entries = list(self.timeline.get_entries_with_bags(self.topic, self.timeline.play_region[0], self.timeline.play_region[1]))[::step]
        total_frames = len(bag_entries)

        self.progress = wx.ProgressDialog('Saving frames from selected region...', 'Saving %d frames from %s...' % (total_frames, self.topic),
                                          maximum=total_frames,
                                          parent=self,
                                          style=wx.PD_CAN_ABORT | wx.PD_APP_MODAL | wx.PD_ELAPSED_TIME | wx.PD_ESTIMATED_TIME | wx.PD_REMAINING_TIME | wx.PD_SMOOTH | wx.PD_AUTO_HIDE)

        self.export_thread = threading.Thread(target=self._run, args=(file_spec, bag_entries))
        self.export_thread.start()
        
    def _run(self, file_spec, bag_entries):
        self.keep_going = True
        
        frame_num = 1
        for i, (bag, entry) in enumerate(bag_entries):
            try:
                topic, msg, t = self.timeline.read_message(bag, entry.position)
                
                pil_img = image_helper.imgmsg_to_pil(msg)
                if pil_img:
                    filename = file_spec % frame_num
                    try:
                        pil_img.save(filename)
                        frame_num += 1
                    except Exception, ex:
                        print >> sys.stderr, 'Error saving frame at %s to disk: %s' % (str(t), str(ex))
            except Exception, ex:
                print >> sys.stderr, 'Error saving frame %d: %s' % (i, str(ex))

            wx.CallAfter(self._update_progress, i)
            if not self.keep_going:
                wx.CallAfter(self.progress.Close)
                break
            
        wx.CallAfter(self.Close)

    def _update_progress(self, i):
        self.keep_going, _ = self.progress.Update(i + 1)

    def _on_cancel(self, event):
        self.Close()
