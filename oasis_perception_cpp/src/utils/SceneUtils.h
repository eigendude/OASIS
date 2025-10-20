/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace OASIS
{
namespace UTILS
{

class SceneUtils
{
public:
  /*!
   * \brief Get byte length of line for grayscale image
   *
   * \param width The image width
   *
   * \return The length of each line in the image
   */
  static unsigned int GetStride(unsigned int width);

  /*!
   * \brief Get length of buffer for an RGBA image of the given size
   *
   * \param width The image width
   * \param height The image height
   *
   * \return The length of the buffer to hold the RGBA image
   */
  static unsigned int GetImageBufferLength(unsigned int width, unsigned int height);

  /*!
   * \brief Calculate the "scene score" of an image
   *
   * The scene score is a value in the range [0.0, 1.0] that indicates the
   * likelihood that an abrupt scene change has occurred.
   *
   * This approach calculates the sum of absolute differences (SAD) to measure
   * the similarity of the two images.
   *
   * Using the SAD value, a mean absolute frame difference difference (MAFD)
   * is computed. The scene score is then given by comparing MAFD values
   * between successive frames.
   *
   * \param previousMafd The MAFD of the previous frame
   * \param currentMafd The MAFD of the current frame
   *
   * \return The scene score
   *
   */
  static float CalcSceneScore(float currentMafd, float previousMafd);

  /*!
   * \brief Get the Mean Absolute Frame Difference
   *
   * \param currentFrame The current frame
   * \param previousFrame The previous frame
   * \param width The image width
   * \param height The image height
   *
   * \return The MAFD
   *
   * \sa CalcSceneScore()
   */
  static float CalcSceneMAFD(const uint8_t* previousFrame,
                             const uint8_t* currentFrame,
                             unsigned int width,
                             unsigned int height);

private:
  /*!
   * \brief Get the Sum of Absolute Differences (a measure of image similarity)
   * between two frames
   *
   * \param src1 The buffer for the first frame
   * \param stride1 The length of each line of the first frame
   * \param src2 The buffer for the second frame
   * \param stride2 The length of each line of the second frame
   * \param width The image width
   * \param height The image height
   *
   * \return The SAD of the two frames
   */
  static uint64_t CalcSceneSAD(const uint8_t* src1,
                               ptrdiff_t stride1,
                               const uint8_t* src2,
                               ptrdiff_t stride2,
                               ptrdiff_t width,
                               ptrdiff_t height);

  /**
   * Clip a float value into the min-max range
   *
   * \param a The value to clip
   * \param min Minimum value of the clip range
   * \param max Maximum value of the clip range
   *
   * \return Clipped value
   */
  static float ClipValue(float value, float min, float max);
};

} // namespace UTILS
} // namespace OASIS
