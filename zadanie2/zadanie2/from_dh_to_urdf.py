from math import sin, cos
import copy


def z_pliku(nazwa_pliku):
    f = open(nazwa_pliku, "r")
    linijki = []
    for line in f:
        linijki.append(line.split())
    f.close()
    return linijki


def z_dh_do_urdf(dane):
    # Trzy pierwsze komorki to wartosci xyz a trzy nastepne to rpy. 
    # Ostatnia komorka to typ laczenia:
    # 0 - fixed
    # 1 - continuous
    # 2 - prismatic
    urdf = [None] * 7
    x = float(dane[2]) * sin(float(dane[0])) * (-1)
    urdf[0] = str(dane[1])
    urdf[1] = str(x)
    urdf[2] = str(float(dane[2]) * cos(float(dane[0])))
    urdf[3] = str(float(dane[0]))
    urdf[4] = "0"
    urdf[5] = str(float(dane[3]))
    laczenie = {
        0: "fixed",
        1: "continuous",
        2: "prismatic"
    }
    urdf[6] = laczenie.get(int(dane[4]))
    return urdf


def stworz_plik(plik_do_zapisu, plik_z_danymi):
    dane_z_pliku = copy.deepcopy(z_pliku(plik_z_danymi))
    f = open(plik_do_zapisu, "w")
    for wartosci in range(1, len(dane_z_pliku)):
        dane_do_urdf = z_dh_do_urdf(dane_z_pliku[wartosci])
        f.write('<joint name="joint_'+str(wartosci)+'" type="'+dane_do_urdf[6]+'">\n')
        f.write('    <parent link="TODO"/>\n')
        f.write('    <child link="TODO"/>\n')
        f.write('    <axis xyz="0 0 1"/>\n')
        f.write('    <origin xyz="'+dane_do_urdf[0]+' '+dane_do_urdf[1]+' '+dane_do_urdf[2]+'" rpy="'+dane_do_urdf[3]+' '+dane_do_urdf[4]+' '+dane_do_urdf[5]+'"/>\n')
        f.write('</joint>\n')
        f.write('\n')
    f.close()


stworz_plik('doZapisu.txt', 'dh.txt')
