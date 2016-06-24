/*
 * Raspberry pi I2C master test function.
 * Created by Juho Iipponen on June 18, 2016.
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
 
 #include <iostream>
 #include "i2cCommunication.h"
 // TODO: Korjaa I2C_respondToRequests() raspille sopivaksi.
 int main()
 {
#ifdef RASPBERRY_PI
	int errorCode;
	
	while(true)
	{
		int numAvail = Wire.available();
//		int randomInt = I2C_requestRandomInt(12, errorCode);
//		std::cout << errorCode << " ";
//		std::cout << randomInt << std::endl;
		sleep(1);
	}
#endif
 }
