import random


def get_priority_from_percentage_difference(percentage_difference):
    if percentage_difference < 10:
        return 0
    else:
        return int(percentage_difference // 10)


def generate_random_floats(size, min_val, max_val):
    random_floats = [random.uniform(min_val, max_val) for _ in range(size)]
    return random_floats


def compare_average_against_control_average(value, average):
    priority = 0
    percentage_difference = abs((value - average) / average) * 100
    print("Value: ", value, " Average: ", average, " Difference: ", percentage_difference)
    if value < average:
        print(f"The value is {percentage_difference:.2f}% less than the average.")
        priority = get_priority_from_percentage_difference(percentage_difference)
        return True, False, percentage_difference, priority
    elif value > average:
        print(f"The value is {percentage_difference:.2f}% greater than the average.")
        # TODO maybe check control here
        priority = get_priority_from_percentage_difference(percentage_difference)
        return True, True, percentage_difference, priority
    else:
        print("The value is equal to the average.")

    return False, percentage_difference, priority


# Example usage:
size = 10
min_val = 1.0
max_val = 200.0

array = generate_random_floats(size, min_val, max_val)
average = 100
array.append(10)
for item in array:
    print("Value: ", item)
    operation, percentage_difference, priority = compare_average_against_control_average(item, average)
    print(operation, percentage_difference, priority)

# Example usage:
# value = 15.5
# result = calculate_result(value)
# print(result)  # Output: 10
