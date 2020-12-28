import csv
import matplotlib
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("parallel.csv") as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        rows = [row for row in reader]
        fig, ax = plt.subplots()
        #ax.plot([row[0] for row in rows], [row[1] for row in rows])
        x = [row[0] for row in rows]
        parallel = [row[1] for row in rows]
        sequential = [row[2] for row in rows]
        ax.plot(x, parallel)
        ax.set(xlabel='Число элементов', ylabel='Время поиска, мкс')
        ax.grid()
        plt.xticks(x, x, rotation='vertical')
        plt.show()
