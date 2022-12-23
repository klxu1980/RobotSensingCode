import random
import math
import numpy as np
import matplotlib.pyplot as plt


def draw_error_distribution(errors):
    plt.plot(errors)
    plt.show()

    err_dist = {}
    for _, err in enumerate(errors):
        err = int(err * 1000)
        if err == 0:
            continue
        if err not in err_dist:
            err_dist[err] = 0
        err_dist[err] += 1
    err_dist = sorted(err_dist.items())

    x = []
    y = []
    for _, err in enumerate(err_dist):
        x.append(err[0] / 1000)
        y.append(err[1] / len(errors))

    plt.plot(x, y)
    plt.show()


def generate_white_noise(count, add_coarse_err=False):
    errors = []
    real = []
    for i in range(count):
        #real.append(i * 0.5)
        real.append(0)
        #real.append(i * i * 0.01)
        errors.append(random.randint(-100, 100) + real[-1])
        if add_coarse_err:
            if i == int(count / 2):
                errors.append(1000) #random.randint(-1000, 1000))
    return real, errors


def generate_gauss_noise(count, add_coarse_err=False):
    errors = []
    real = []
    for i in range(count):
        real.append(i * 0.5)
        real.append(0)
        #real.append(i * i * 0.01)
        errors.append(random.randint(-100, 100) + real[-1])
        if add_coarse_err:
            if i == int(count / 2):
                errors.append(1000) #random.randint(-1000, 1000))
    return real, errors


def show_white_noise(count):
    errors = generate_white_noise(count)
    draw_error_distribution(errors)


def filter_noise(real, errors, K):
    y = 0
    rst = []
    for _, err in enumerate(errors):
        y = K * y + (1 - K) * err
        rst.append(y)

    plt.plot(errors)
    plt.plot(real)
    plt.plot(rst, 'r')
    plt.show()


def midvalue_filter_noise(real, errors, window_size):
    rst = []
    samples = []
    for i, err in enumerate(errors):
        samples.append(err)
        if len(samples) <= window_size:
            continue
        samples.pop(0)

        sort = samples.copy()
        sort = sorted(sort)
        rst.append(sort[int(window_size / 2)])

    plt.plot(errors)
    plt.plot(real)
    plt.plot(rst)
    plt.show()


def filter_without_min_max(real, errors, window_size):
    rst = []
    samples = []
    for i, err in enumerate(errors):
        samples.append(err)
        if len(samples) <= window_size:
            continue
        samples.pop(0)

        data = sorted(samples)
        data.pop(-1)
        data.pop(0)

        avg = np.average(np.array(data))
        rst.append(avg)

    plt.plot(errors)
    plt.plot(real)
    plt.plot(rst)
    plt.show()


def filter_by_average(real, errors, window_size):
    rst = []
    samples = []
    for i, err in enumerate(errors):
        samples.append(err)
        if len(samples) > window_size:
            samples.pop(0)

        avg = np.average(np.array(samples))
        rst.append(avg)

    plt.plot(errors)
    plt.plot(real)
    plt.plot(rst)
    plt.show()


#def filter_by_


if __name__ == '__main__':
    #_, errors = generate_white_noise(count=500000)
    #draw_error_distribution(errors)

    #real, errors = generate_white_noise(count=500, add_coarse_err=False)
    #filter_noise(real, errors, 0.99)
    #midvalue_filter_noise(real, errors, window_size=51)
    #filter_without_min_max(real, errors, window_size=100)
    #filter_by_average(real, errors, window_size=100)

    errors = []
    for i in range(5000000):
        errors.append(random.gauss(0, 10))
    draw_error_distribution(errors)