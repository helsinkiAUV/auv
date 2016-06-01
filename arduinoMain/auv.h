/*
 * Main AUV header
 * Created by Juho Iipponen on March 22, 2016.
 *
 * This file is part of the University of Helsinki AUV source code.
 *
 * The AUV source code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * The AUV source code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the source code.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#ifndef AUV_H_
#define AUV_H_

#if !defined(RASPBERRY_PI) && !defined(SIMULATOR) && !defined(ARDUINO)
  #define ARDUINO
#endif

#ifdef ARDUINO
  #define ARDUINO_LEFT // Change this line, when compiling for each of the two Arduinos.
#endif

#endif
