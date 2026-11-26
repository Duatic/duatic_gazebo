/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdlib>
#include <cstring>
#include <string_view>
#include <vector>

#include <QGuiApplication>
#include <QLabel>
#include <QSocketNotifier>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#include <QX11Info>
#endif

#include <pluginlib/class_list_macros.hpp>
#include <sys/uio.h>
#include <xcb/xcb.h>
#include <xcb/xcbext.h>
// Jazzy ships only the .h header, later distros deprecate it.
#if __has_include(<rqt_gui_cpp/plugin.hpp>)
#include <rqt_gui_cpp/plugin.hpp>
#else
#include <rqt_gui_cpp/plugin.h>
#endif

namespace duatic_gazebo
{

// Instance name in WM_CLASS of the Gazebo GUI main window.
constexpr std::string_view GAZEBO_WM_INSTANCE = "gz-sim-gui";

// The X connection Qt itself uses, which owns the windows the Gazebo window gets embedded in.
xcb_connection_t* qtConnection()
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  return QX11Info::connection();
#else
  return qGuiApp->nativeInterface<QNativeInterface::QX11Application>()->connection();
#endif
}

// Adds the window to, or removes it from, the save-set of Qt's connection, so the X server moves it back to the
// root instead of destroying it when rqt exits without shutdownPlugin(), as on Ctrl+C or a crash.
// XFixes request, as the core one reparents to rqt's window manager frame, which dies with rqt. Sent by hand
// because libxcb-xfixes has no rosdep key.
void changeSaveSet(bool insert, xcb_window_t window)
{
  static xcb_extension_t xfixes = { "XFIXES", 0 };
  static const xcb_protocol_request_t request = { 2, &xfixes, 1 /* XFixesChangeSaveSet */, 1 };
  xcb_connection_t* connection = qtConnection();
  const xcb_query_extension_reply_t* extension = xcb_get_extension_data(connection, &xfixes);
  if (extension == nullptr || !extension->present) {
    return;
  }
  struct
  {
    uint8_t major_opcode;
    uint8_t minor_opcode;
    uint16_t length;
    uint8_t mode;    // 0 insert, 1 delete
    uint8_t target;  // 0 nearest ancestor, 1 root
    uint8_t map;     // 0 map, 1 unmap
    uint8_t pad;
    xcb_window_t window;
  } body{};
  body.mode = insert ? 0 : 1;
  body.target = 1;
  body.window = window;
  // xcb_send_request() fills the header and uses the two iovecs in front of the request for itself.
  struct iovec parts[4] = {};
  parts[2].iov_base = &body;
  parts[2].iov_len = sizeof(body);
  xcb_send_request(connection, 0, &parts[2], &request);
}

// Shows the Gazebo GUI window inside an rqt panel by reparenting it. Gazebo keeps rendering straight to the X
// server, so no frames pass through rqt.
class RqtGazeboView : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  void initPlugin(qt_gui_cpp::PluginContext& context) override;
  void shutdownPlugin() override;

private:
  xcb_atom_t internAtom(const char* name);
  void processEvents();
  void findWindow();
  void track(xcb_window_t window);
  void advance();
  void fitToContainer(const xcb_configure_notify_event_t& current);
  void release(bool window_alive);

  QWidget* widget_{ nullptr };
  QLabel* label_{ nullptr };
  QWidget* container_{ nullptr };
  QSocketNotifier* notifier_{ nullptr };

  // Own connection: its events wake this plugin only, and Qt's event masks stay untouched.
  xcb_connection_t* connection_{ nullptr };
  xcb_window_t root_{ XCB_NONE };
  xcb_atom_t client_list_atom_{ XCB_NONE };
  xcb_atom_t wm_state_atom_{ XCB_NONE };
  // The Gazebo window on its way into the panel, XCB_NONE while waiting for one.
  xcb_window_t window_{ XCB_NONE };
  enum class Stage
  {
    APPEARING,    // the window manager is still setting the window up
    WITHDRAWING,  // asked the window manager to let go of it
    EMBEDDED
  } stage_{ Stage::APPEARING };
};

void RqtGazeboView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  widget_->setWindowTitle("Gazebo");
  label_ = new QLabel("Waiting for the Gazebo window...", widget_);
  label_->setAlignment(Qt::AlignCenter);
  auto* layout = new QVBoxLayout(widget_);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(label_);
  context.addWidget(widget_);

  if (QGuiApplication::platformName() != "xcb") {
    label_->setText("Embedding the Gazebo window needs an X11 session.");
    return;
  }

  int screen_number = 0;
  connection_ = xcb_connect(nullptr, &screen_number);
  if (xcb_connection_has_error(connection_)) {
    label_->setText("Could not connect to the X server.");
    xcb_disconnect(connection_);
    connection_ = nullptr;
    return;
  }

  xcb_screen_iterator_t screens = xcb_setup_roots_iterator(xcb_get_setup(connection_));
  for (; screen_number > 0; --screen_number) {
    xcb_screen_next(&screens);
  }
  root_ = screens.data->root;

  client_list_atom_ = internAtom("_NET_CLIENT_LIST");
  wm_state_atom_ = internAtom("WM_STATE");

  // The window manager rewrites _NET_CLIENT_LIST on the root whenever a top-level window appears or goes away.
  const uint32_t root_events = XCB_EVENT_MASK_PROPERTY_CHANGE;
  xcb_change_window_attributes(connection_, root_, XCB_CW_EVENT_MASK, &root_events);

  notifier_ = new QSocketNotifier(xcb_get_file_descriptor(connection_), QSocketNotifier::Read, this);
  connect(notifier_, &QSocketNotifier::activated, this, &RqtGazeboView::processEvents);

  findWindow();
  processEvents();
}

xcb_atom_t RqtGazeboView::internAtom(const char* name)
{
  xcb_intern_atom_reply_t* reply =
      xcb_intern_atom_reply(connection_, xcb_intern_atom(connection_, 0, std::strlen(name), name), nullptr);
  if (reply == nullptr) {
    return XCB_NONE;
  }
  const xcb_atom_t atom = reply->atom;
  std::free(reply);
  return atom;
}

void RqtGazeboView::shutdownPlugin()
{
  if (connection_ == nullptr) {
    return;
  }
  delete notifier_;
  notifier_ = nullptr;
  if (window_ != XCB_NONE) {
    release(true);
  }
  xcb_disconnect(connection_);
  connection_ = nullptr;
}

void RqtGazeboView::processEvents()
{
  // Replies read in findWindow() can queue further events without the socket becoming readable again, so drain
  // until the queue is empty.
  while (xcb_generic_event_t* event = xcb_poll_for_event(connection_)) {
    switch (event->response_type & ~0x80) {
      case XCB_PROPERTY_NOTIFY: {
        const auto* property = reinterpret_cast<xcb_property_notify_event_t*>(event);
        if (window_ == XCB_NONE && property->window == root_ && property->atom == client_list_atom_) {
          findWindow();
        } else if (property->window == window_ && property->atom == wm_state_atom_) {
          advance();
        }
        break;
      }
      case XCB_REPARENT_NOTIFY: {
        if (reinterpret_cast<xcb_reparent_notify_event_t*>(event)->window == window_) {
          advance();
        }
        break;
      }
      case XCB_MAP_NOTIFY: {
        if (reinterpret_cast<xcb_map_notify_event_t*>(event)->window == window_) {
          advance();
        }
        break;
      }
      case XCB_CONFIGURE_NOTIFY: {
        const auto* configure = reinterpret_cast<xcb_configure_notify_event_t*>(event);
        if (stage_ == Stage::EMBEDDED && configure->window == window_) {
          fitToContainer(*configure);
        }
        break;
      }
      case XCB_DESTROY_NOTIFY: {
        const auto* destroy = reinterpret_cast<xcb_destroy_notify_event_t*>(event);
        if (window_ != XCB_NONE && destroy->window == window_) {
          release(false);
          findWindow();
        }
        break;
      }
      default:
        break;
    }
    std::free(event);
  }
  xcb_flush(connection_);

  // A broken connection stays readable forever and would spin the event loop.
  if (xcb_connection_has_error(connection_)) {
    notifier_->setEnabled(false);
    label_->setText("Lost the connection to the X server.");
  }
}

void RqtGazeboView::findWindow()
{
  xcb_get_property_reply_t* list = xcb_get_property_reply(
      connection_, xcb_get_property(connection_, 0, root_, client_list_atom_, XCB_ATOM_WINDOW, 0, 4096), nullptr);
  if (list == nullptr) {
    return;
  }
  const auto* windows = static_cast<const xcb_window_t*>(xcb_get_property_value(list));
  const int count = xcb_get_property_value_length(list) / static_cast<int>(sizeof(xcb_window_t));

  // Send every WM_CLASS request before reading a reply: one round trip for the whole list.
  std::vector<xcb_get_property_cookie_t> cookies;
  cookies.reserve(count);
  for (int i = 0; i < count; ++i) {
    cookies.push_back(xcb_get_property(connection_, 0, windows[i], XCB_ATOM_WM_CLASS, XCB_ATOM_STRING, 0, 64));
  }
  for (int i = 0; i < count; ++i) {
    if (window_ != XCB_NONE) {
      xcb_discard_reply(connection_, cookies[i].sequence);
      continue;
    }
    xcb_get_property_reply_t* wm_class = xcb_get_property_reply(connection_, cookies[i], nullptr);
    if (wm_class == nullptr) {
      continue;
    }
    // WM_CLASS holds two null-terminated strings: instance, then class.
    const auto* value = static_cast<const char*>(xcb_get_property_value(wm_class));
    const std::size_t length = xcb_get_property_value_length(wm_class);
    const bool is_gazebo = std::string_view(value, strnlen(value, length)) == GAZEBO_WM_INSTANCE;
    std::free(wm_class);
    if (is_gazebo) {
      track(windows[i]);
    }
  }
  std::free(list);
}

void RqtGazeboView::track(xcb_window_t window)
{
  // Checked, so a window that is already gone is skipped instead of tracked without a DestroyNotify to follow.
  const uint32_t window_events = XCB_EVENT_MASK_STRUCTURE_NOTIFY | XCB_EVENT_MASK_PROPERTY_CHANGE;
  xcb_generic_error_t* error = xcb_request_check(
      connection_, xcb_change_window_attributes_checked(connection_, window, XCB_CW_EVENT_MASK, &window_events));
  if (error != nullptr) {
    std::free(error);
    return;
  }
  window_ = window;
  stage_ = Stage::APPEARING;
  advance();
}

void RqtGazeboView::advance()
{
  if (stage_ == Stage::EMBEDDED) {
    return;
  }
  const xcb_query_tree_cookie_t tree_cookie = xcb_query_tree(connection_, window_);
  const xcb_get_property_cookie_t state_cookie =
      xcb_get_property(connection_, 0, window_, wm_state_atom_, wm_state_atom_, 0, 2);
  const xcb_get_window_attributes_cookie_t attributes_cookie = xcb_get_window_attributes(connection_, window_);
  xcb_query_tree_reply_t* tree = xcb_query_tree_reply(connection_, tree_cookie, nullptr);
  xcb_get_property_reply_t* state = xcb_get_property_reply(connection_, state_cookie, nullptr);
  xcb_get_window_attributes_reply_t* attributes =
      xcb_get_window_attributes_reply(connection_, attributes_cookie, nullptr);
  const bool on_root = tree != nullptr && tree->parent == root_;
  // WM_STATE is missing or Withdrawn (0) while the window manager does not show the window.
  const bool managed = state != nullptr && xcb_get_property_value_length(state) > 0 &&
                       *static_cast<const uint32_t*>(xcb_get_property_value(state)) != 0;
  const bool on_screen = attributes != nullptr && attributes->map_state == XCB_MAP_STATE_VIEWABLE;
  const bool gone = tree == nullptr || state == nullptr || attributes == nullptr;
  std::free(tree);
  std::free(state);
  std::free(attributes);
  if (gone) {
    return;
  }

  // A window taken before the window manager has put it on screen stays blank once embedded.
  if (stage_ == Stage::APPEARING && managed && on_screen) {
    // A window manager only lets go of a window that is withdrawn the ICCCM way: unmapped, plus a synthetic
    // UnmapNotify on the root. Reparenting a window it still manages leaves its frame behind.
    xcb_unmap_window(connection_, window_);
    union
    {
      xcb_unmap_notify_event_t unmap;
      char raw[32];
    } event{};
    event.unmap.response_type = XCB_UNMAP_NOTIFY;
    event.unmap.event = root_;
    event.unmap.window = window_;
    xcb_send_event(connection_, 0, root_, XCB_EVENT_MASK_SUBSTRUCTURE_NOTIFY | XCB_EVENT_MASK_SUBSTRUCTURE_REDIRECT,
                   event.raw);
    xcb_flush(connection_);
    stage_ = Stage::WITHDRAWING;
  } else if (stage_ == Stage::WITHDRAWING && !managed && on_root) {
    changeSaveSet(true, window_);
    container_ = QWidget::createWindowContainer(QWindow::fromWinId(window_), widget_);
    widget_->layout()->addWidget(container_);
    label_->hide();
    stage_ = Stage::EMBEDDED;
  }
}

void RqtGazeboView::fitToContainer(const xcb_configure_notify_event_t& current)
{
  // Gazebo applies its saved window size while it starts, which Qt does not see on a foreign window.
  const QWidget* parent = container_->nativeParentWidget();
  if (parent == nullptr) {
    return;
  }
  const qreal scale = container_->devicePixelRatioF();
  const QPoint position = container_->mapTo(parent, QPoint(0, 0)) * scale;
  const QSize size = container_->size() * scale;
  if (current.x == position.x() && current.y == position.y() && current.width == size.width() &&
      current.height == size.height()) {
    return;
  }
  const uint32_t geometry[] = { static_cast<uint32_t>(position.x()), static_cast<uint32_t>(position.y()),
                                static_cast<uint32_t>(size.width()), static_cast<uint32_t>(size.height()) };
  xcb_configure_window(connection_, window_,
                       XCB_CONFIG_WINDOW_X | XCB_CONFIG_WINDOW_Y | XCB_CONFIG_WINDOW_WIDTH | XCB_CONFIG_WINDOW_HEIGHT,
                       geometry);
}

void RqtGazeboView::release(bool window_alive)
{
  delete container_;
  container_ = nullptr;

  if (window_alive) {
    // Qt unmaps the window when the container goes. Once that has reached the server, hand the window back to
    // the desktop so the Gazebo GUI outlives this panel.
    if (stage_ == Stage::EMBEDDED) {
      changeSaveSet(false, window_);
    }
    QGuiApplication::sync();
    xcb_reparent_window(connection_, window_, root_, 0, 0);
    xcb_map_window(connection_, window_);
    // Wait for the server: Qt's connection is separate, and it may destroy the old parent window right after.
    std::free(xcb_get_input_focus_reply(connection_, xcb_get_input_focus(connection_), nullptr));
  }

  window_ = XCB_NONE;
  label_->show();
}

}  // namespace duatic_gazebo

PLUGINLIB_EXPORT_CLASS(duatic_gazebo::RqtGazeboView, rqt_gui_cpp::Plugin)

#include "rqt_gazebo_view.moc"
