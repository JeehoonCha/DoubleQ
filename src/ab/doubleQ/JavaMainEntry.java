package ab.doubleQ;

import py4j.GatewayServer;

/*****************************************************************************
 ** ANGRYBIRDS AI AGENT FRAMEWORK
 ** Copyright (c) 2014, XiaoYu (Gary) Ge, Jochen Renz,Stephen Gould,
 **  Sahan Abeyasinghe,Jim Keys,  Andrew Wang, Peng Zhang
 ** All rights reserved.
**This work is licensed under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
**To view a copy of this license, visit http://www.gnu.org/licenses/
 *****************************************************************************/

public class JavaMainEntry {
	// the entry of the software.
	public static void main(String args[])
	{
		JavaShootingAgent javaAgent = new JavaShootingAgent();
		GatewayServer server = new GatewayServer(javaAgent);
		server.start();
	}
}