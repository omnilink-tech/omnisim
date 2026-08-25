// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#ifndef OM_GL_WINDOW_HPP
#define OM_GL_WINDOW_HPP

//
// Description: renderer-agnostic Qt native window owning the GL context + the render/resize
// event plumbing (D1.3, WREN-deletion runbook C8). Everything here survives the WREN
// deletion; the WREN-specific rendering lives in the OmWrenWindow subclass until D1.4
// deletes it and OmView3D re-bases here.
//

#include <QtGui/QImage>
#include <QtGui/QWindow>

class OmMultimediaStreamingServer;

class OmGlWindow : public QWindow {
  Q_OBJECT;

public:
  static OmGlWindow *instance();
  static void flipAndScaleDownImageBuffer(const unsigned char *source, unsigned char *destination, int sourceWidth,
                                          int sourceHeight, int scaleDownFactor);

  explicit OmGlWindow();
  virtual ~OmGlWindow();

  // No-window headless mode (core-evolution-plan.md, Phase Q1): realize the (hidden) native
  // window and run the full renderer initialization so node finalize (createWrenObjects) and
  // camera-sensor rendering work with no GUI on top. Public counterpart of the protected,
  // event-driven initialize(); safe to call more than once.
  void initializeForHeadless() { initialize(); }

  virtual QSize minimumSize() const;
  QSize sizeHint() const;

  // return immediate screenshot. Virtual: subclasses source pixels from their own render
  // target (OmWrenWindow reads the WREN GL buffer; OmView3D prefers the wgpu render target
  // when wgpu presented the last main-view frame). The base has no renderer, so no pixels.
  virtual QImage grabWindowBufferNow();

  // The renderer-independent half of the old updateWrenViewportDimensions() (brief sect. 6.3):
  // publish this window's device-pixel ratio to the movie recorder, which sizes its capture
  // buffers from it. OmWrenWindow::updateWrenViewportDimensions() layers the WREN viewport
  // update on top and calls this; after D1.4 this is all that remains of that path.
  void updateScreenPixelRatio();

  void setVideoStreamingServer(OmMultimediaStreamingServer *streamingServer);

public slots:
  virtual void renderLater();

signals:
  void resized();

protected:
  virtual void initialize();
  virtual void renderNow(bool culling = true, bool offScreen = false);
  virtual void resizeWren(int width, int height);
  bool event(QEvent *event) override;
  // web-stream (--stream mjpeg) server, exposed so OmView3D's wgpu main-view path can feed it a
  // frame — the WREN-arm feed lives inside OmWrenWindow::renderNow, which wgpu skips entirely.
  class OmMultimediaStreamingServer *videoStreamingServer() const { return mVideoStreamingServer; }

private:
  static OmGlWindow *cInstance;

  bool mUpdatePending;

  OmMultimediaStreamingServer *mVideoStreamingServer;

private slots:
  void feedMultimediaStreamer();
};

#endif
