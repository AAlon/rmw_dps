import sys
import os

RESOURCES_FILE_NAME = 'resources.txt'
# Ignore the first few data points so we can be sure the system is in steady state
SKIPPED_LINES = 3

THRESHOLD_CONFIGURATION = {
    'rss': 12000,
    'vsz': 10 ** 6,
}

def main(log_dir_path):
    print('Checking performance tests results...')
    resources_file_path = os.path.join(log_dir_path, RESOURCES_FILE_NAME)
    with open(resources_file_path) as resources_file:
        lines = [ line.strip().split() for line in resources_file.readlines() if line.strip() ]

    for key, value in THRESHOLD_CONFIGURATION.items():
        _assert_resources_item_threshold(lines, key, value)
    print('Performance check finished successfully.')

def _assert_resources_item_threshold(lines, item, threshold):
    descriptor = lines[0]
    item_index = descriptor.index([ x for x in descriptor if item in x][0])
    for line in lines[SKIPPED_LINES:]:
        measurement = float(line[item_index])
        if measurement > threshold:
            print('FAILED: Threshold breached for item "%s": %f > %f\nLine: "%s"' % \
              (item, measurement, threshold, '\t'.join(line), ))
            sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise ValueError('Usage: perf_chcker.py <log directory path>')
    main(sys.argv[1])
