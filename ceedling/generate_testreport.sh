#!/bin/bash

# @echo off
ceedling test:all
ceedling gcov:all
# gcovr --root=../main --filter=../main --html --verbose -o build/artifacts/coverage_report.html
# gcovr --root=. --filter=../main --html --html-details --csv --json --json-summary --txt -o build/artifacts/coverage_report.html

# Detailed HTML report
gcovr --root=. --filter=../main --html-details -o build/artifacts/coverage_report.html

# Plain text summary
gcovr --root=. --filter=../main --txt -o build/artifacts/coverage_report.txt

# CSV
gcovr --root=. --filter=../main --csv -o build/artifacts/coverage_report.csv

# JSON
gcovr --root=. --filter=../main --json -o build/artifacts/coverage_report.json

# JSON Summary
gcovr --root=. --filter=../main --json-summary -o build/artifacts/coverage_summary.json

