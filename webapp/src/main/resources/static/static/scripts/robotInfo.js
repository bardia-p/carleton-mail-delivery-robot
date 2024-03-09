const robotInfoButton = $("#robotInfo-submit");


/**
 * The JavaScript AJAX call for when robot information is retrieved.
 */
robotInfoButton.click((e) => {
    e.preventDefault();
    const splitRef = window.location.href.split("/");
    const id = splitRef[splitRef.length - 1];
    $.ajax({
        type: "GET",
        url: `/api/v1/getRobotDeliveries/${id}`,
        success: (res) => {
            if (res) {
                window.location.href = `/robot/${res["robotId"]}`;
            }
        },
        error: (error) => {
            setToast("error", "Something went wrong", "Could not view robot info at this time");
            console.error(error);
        }
    });
});