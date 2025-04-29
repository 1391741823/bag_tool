#  融合多个bag包文件工具

##  运行终端打开：
python3 merge_bag.py -v <bagtarget.bag> <bagname1.bag> <bagname2.bag>

### 上面程序说明 
在其中，第一个bag名字为你的目标bag名字，后面的两个包为你的bag包的名字，将其融合成一个bag包数据如：

python3 merge_bag.py -v total.bag Retail_street.bag setic_image.bag 

(将Retail_street.bag与setic_image.bag融合进total.bag中)

##  将2个bag里面某些topic合并到一个bag包里面
python3 merge_bag.py -v --topic ' <topic_name1 topic_name2 topic_name3> ' <bagtarget.bag> <bagname1.bag> <bagname2.bag>

### 上面程序说明 
将<bagname1.bag>中的topic_name1 topic_name2 与<bagname2.bag>中的 topic_name3融合进<bagtarget.bag>中
