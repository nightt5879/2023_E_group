
#冒泡排序
def bubble_sort(l):
    for i in range(len(l)-1):
        for j in range(len(l)-1-i):
            if l[j] > l[j+1]:
                l[j],l[j+1] = l[j+1],l[j]
    return l

#测试
l = [1,3,2,5,4]
print(bubble_sort(l))
