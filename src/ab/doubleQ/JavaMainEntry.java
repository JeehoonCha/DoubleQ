package ab.doubleQ;

import py4j.GatewayServer;

import java.net.*;

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
		try{
			InetAddress host = Inet6Address.getLocalHost();
			GatewayServer server = new GatewayServer(
					javaAgent,
					GatewayServer.DEFAULT_PORT,
					GatewayServer.DEFAULT_PYTHON_PORT,
					host,
					null,
					0,
					0,
					null);
			server.start();
			System.out.println("GatewayServer for " + javaAgent.getClass().getName() + " started on " + host.toString()
					+ ":" + String.valueOf(GatewayServer.DEFAULT_PORT));
		} catch (UnknownHostException e) {
		}

	}
}