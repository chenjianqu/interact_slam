#!/usr/bin/env python
# -*- coding: utf-8 -*-

#########################################################
# Copyright (C) 2022, Chen Jianqu, Shanghai University
#
# This file is part of interact_slam.
#
# Licensed under the MIT License;
# you may not use this file except in compliance with the License.
#########################################################



import rospy
from interact_slam.srv import *

import cv2
import json
import urllib2
import urllib
import re
import string


keyWords=["人","自行车","汽车","摩托车","飞机",
		  	"公共汽车","火车","卡车","船","红绿灯",
		  "消防栓","停车标志","停车表","长凳","鸟",
		  	"猫","狗","马","羊","牛",
		  	"象","熊","斑马","长颈鹿","背包",
		  	"伞","手提包","领带","手提箱","飞盘",
			"滑雪板","滑雪板","运动球","风筝","棒球棒",
			"棒球手套","滑板","冲浪板","网球拍","瓶子",
		  	"酒杯","杯子","叉子","刀子","勺子",
		  	"碗","香蕉","苹果","三明治","橘子",
		  	"花椰菜","胡萝卜","热狗","披萨","甜甜圈",
		  	"蛋糕","椅子","沙发","盆栽","床",
		  	"餐桌","马桶","电视","笔记本","鼠标",
		  	"遥控器","键盘","手机","微波炉","烤箱",
			"烤面包机","水槽","冰箱","书","时钟",
		  	"花瓶","剪刀","泰迪熊","吹风机","牙刷"
		 ]

fileName="/media/chen/chen/Robot/slam_ws/data/InstanceMat.exr";
instVector=cv2.imread(fileName,-1)


def chat(text):
	BOT_NAME='系统'
	BOT_SELF_NAME='本系统'
	x = urllib.quote(text)
	link = urllib2.urlopen(
		"http://nlp.xiaoi.com/robot/webrobot?&callback=__webrobot_processMsg&data=%7B%22sessionId%22%3A%22ff725c236e5245a3ac825b2dd88a7501%22%2C%22robotId%22%3A%22webbot%22%2C%22userId%22%3A%227cd29df3450745fbbdcf1a462e6c58e6%22%2C%22body%22%3A%7B%22content%22%3A%22" + x + "%22%7D%2C%22type%22%3A%22txt%22%7D")
	html_doc = link.read()
	reply_list = re.findall(r'\"content\":\"(.+?)\\r\\n\"', html_doc)
	reply=str(reply_list[-1])
	reply=reply.replace("小i机器人",BOT_NAME)
	reply=reply.replace("小i",BOT_SELF_NAME)
	return reply

def Handle(text):
	result=''
	key=''
	index=0
	for (i,k) in enumerate(keyWords):
		if(k in text): #存在关键词
			key=k
			index=i
			break
	if(key!=''):
		insts=[]
		for inst in instVector:
			if(int(inst[0])==index):
				insts.append(((inst[2]+inst[3])/2,(inst[4]+inst[5])/2,(inst[6]+inst[7])/2))
		print(insts)
		if(len(insts)==0):
			result="实例地图中未找到"+key
		else:
			if("拿" in text):
				result="发现"+str(len(insts))+"个"+key+"实例，正在导航到：\n"+str(insts[0])
			else:
				result="发现"+str(len(insts))+"个"+key+"实例:\n"+str(insts)
	
	if(("实例" in text or "东西" in text) and ("所有" in text or "全部" in text)):
		result+="当前所有的实例如下：\n"
		instCurrent={}
		for inst in instVector:
			clsIndex=int(inst[0])
			k=keyWords[clsIndex]
			n=instCurrent.get(k,0)
			instCurrent[k]=n+1
		for k,v in instCurrent.items():
			result+=str(k)+":"+str(v)+"\n"
			

	if(result==""):
		#result="无法理解"
		result=chat(text)
		if("资讯" in result):
			result="行吧"
		if("你可能关心" in result):
			result="哦"
		if(result==""):
			result="无法理解"
			
	return result
	

	
def UpdateInstance(event):
	global instVector
	fileName="/media/chen/chen/Robot/slam_ws/data/InstanceMat.exr";
	instVector=cv2.imread(fileName,-1);
	#print(instVector)

 
def CallBack(req):
	s=req.request
	rospy.loginfo("请求："+s)
	result=Handle(s)
	rospy.loginfo("结果："+result)
	return OrderResponse(result) #返回应答数据包（这里传进去的参数按.srv文件的顺序填写）

def OrderServices():
	rospy.init_node('Order_Service_Node')
	s = rospy.Service('OrderService',Order,CallBack)
	
	rospy.Timer(rospy.Duration(10),UpdateInstance)
	
	rospy.loginfo("命令服务节点已启动")
	rospy.spin()
	rospy.loginfo("命令服务节点已关闭")

if __name__ == "__main__":
	OrderServices()