import ybc_face
import ybc_box
pic = ybc_box.fileopenbox()
res = ybc_face.info(pic)
ybc_box.msgbox(f'请告诉我，我的颜值：{res}, {pic}。')
